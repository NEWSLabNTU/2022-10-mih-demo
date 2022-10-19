use crate::{color_sampling::sample_rgb, config::Config, message as msg};
use anyhow::Result;
use async_std::task::spawn_blocking;
use futures::prelude::*;
use nalgebra as na;
use opencv::{
    core::{add_weighted, Point2f, Point2i, Rect, Scalar, Size, CV_32FC3},
    highgui,
    imgproc::{self, FILLED, INTER_LINEAR, LINE_8},
    prelude::*,
};
use palette::{Hsv, IntoColor, RgbHue, Srgb};
use rayon::prelude::*;
use std::{
    collections::HashMap,
    num::NonZeroUsize,
    ops::RangeInclusive,
    time::{Duration, Instant},
};

const INTERVAL: Duration = Duration::from_millis(34);

pub async fn start(
    config: &Config,
    stream: impl Stream<Item = msg::OpencvMessage> + Unpin + Send,
) -> Result<()> {
    let Config {
        otobrite_image_hw,
        kneron_image_hw,
        otobrite_raw_present_size,
        otobrite_fused_present_size,
        kneron_det_present_size,
        kneron_fused_present_size,
        otobrite_distance_range,
        kneron_distance_range,
        otobrite_image_roi_tlbr,
        kneron_image_roi_tlbr,
        otobrite_hue_range,
        ..
    } = *config;

    let otobrite_distance_range = {
        let [min, max] = otobrite_distance_range;
        min..=max
    };
    let kneron_distance_range = {
        let [min, max] = kneron_distance_range;
        min..=max
    };

    let (tx, rx) = flume::bounded(2);

    let forward_future = stream.map(Ok).forward(tx.into_sink()).map(|_result| ());
    let handle_future = spawn_blocking(move || {
        use flume::RecvTimeoutError as E;

        let mut state = {
            let convert_hw = |[h, w]: [NonZeroUsize; 2]| [h.get(), w.get()];
            let tlbr_to_rect = |[t, l, b, r]: [usize; 4]| -> Rect {
                let width = r - l;
                let height = b - t;
                Rect {
                    x: l as i32,
                    y: t as i32,
                    width: width as i32,
                    height: height as i32,
                }
            };

            let otobrite_image_hw = convert_hw(otobrite_image_hw);
            let kneron_image_hw = convert_hw(kneron_image_hw);
            let otobrite_image_roi = tlbr_to_rect(otobrite_image_roi_tlbr);
            let kneron_image_roi = tlbr_to_rect(kneron_image_roi_tlbr);

            let kneron_image = make_zero_mat(kneron_image_hw);
            let otobrite_image = make_zero_mat(otobrite_image_hw);

            State {
                kneron_det_image: kneron_image.clone(),
                kneron_fused_image: kneron_image,
                kneron_det_present_size,
                kneron_fused_present_size,
                otobrite_raw_image: otobrite_image.clone(),
                otobrite_fused_image: otobrite_image,
                otobrite_image_hw,
                otobrite_raw_present_size,
                otobrite_fused_present_size,
                kneron_image_hw,
                otobrite_distance_range,
                kneron_distance_range,
                otobrite_image_roi,
                kneron_image_roi,
                otobrite_hue_range,
            }
        };
        let mut until = Instant::now() + INTERVAL;

        loop {
            match rx.recv_deadline(until) {
                Ok(msg) => {
                    state.update(msg)?;

                    if Instant::now() < until {
                        continue;
                    }
                }
                Err(E::Disconnected) => break,
                Err(E::Timeout) => {}
            }

            state.step()?;

            let next_until = until + INTERVAL;
            let now = Instant::now();
            until = if now < next_until { next_until } else { now };
        }

        anyhow::Ok(())
    });

    futures::try_join!(forward_future.map(|()| anyhow::Ok(())), handle_future)?;
    Ok(())
}

struct State {
    kneron_det_image: Mat,
    kneron_fused_image: Mat,
    kneron_det_present_size: usize,
    kneron_fused_present_size: usize,
    otobrite_raw_image: Mat,
    otobrite_fused_image: Mat,
    otobrite_image_hw: [usize; 2],
    otobrite_raw_present_size: usize,
    otobrite_fused_present_size: usize,
    kneron_image_hw: [usize; 2],
    otobrite_distance_range: RangeInclusive<f32>,
    kneron_distance_range: RangeInclusive<f32>,
    otobrite_image_roi: Rect,
    kneron_image_roi: Rect,
    otobrite_hue_range: [f32; 2],
}

impl State {
    fn step(&mut self) -> Result<()> {
        highgui::imshow("Otobrite Image", &self.otobrite_raw_image)?;
        highgui::imshow("Kneron Detection", &self.kneron_det_image)?;
        highgui::imshow("Kneron Camera + Point Cloud", &self.kneron_fused_image)?;
        highgui::imshow("Otobrite Camera + Point Cloud", &self.otobrite_fused_image)?;
        let _key = highgui::wait_key(1)?;

        Ok(())
    }

    fn update(&mut self, msg: msg::OpencvMessage) -> Result<()> {
        use msg::OpencvMessage as M;

        match msg {
            M::Otobrite(msg) => self.update_otobrite(msg)?,
            M::Kneron(msg) => self.update_kneron(msg)?,
        }

        Ok(())
    }

    fn update_otobrite(&mut self, msg: msg::OtobriteMessage) -> Result<()> {
        let msg::OtobriteMessage { image, assocs } = msg;

        let orig_image: Mat = image.unwrap_or_else(|| make_zero_mat(self.otobrite_image_hw));
        let otobrite_raw_image = {
            let canvas: Mat = orig_image.clone();

            // Crop
            let canvas = Mat::roi(&canvas, self.otobrite_image_roi)?;

            // Scale image
            let canvas = {
                let target_size = self.otobrite_raw_present_size as f64;
                let fx = target_size / canvas.cols() as f64;
                let fy = target_size / canvas.rows() as f64;
                let scale = fx.min(fy);

                let mut out = Mat::default();
                imgproc::resize(
                    &canvas,
                    &mut out,
                    Size::default(),
                    scale,
                    scale,
                    INTER_LINEAR,
                )?;
                out
            };

            canvas
        };
        let otobrite_fused_image = {
            let canvas: Mat = orig_image;

            // Change opacity
            let mut canvas = {
                let mut output = Mat::default();
                add_weighted(&canvas, 0.5, &canvas, 0.0, 0.0, &mut output, -1)?;
                output
            };

            // Draw points
            if let Some(assocs) = assocs {
                let pixels: HashMap<_, _> = assocs
                    .par_iter()
                    .filter_map(|assoc| {
                        let distance =
                            na::distance(&na::Point3::origin(), &assoc.pcd_point.position);
                        self.otobrite_distance_range
                            .contains(&distance)
                            .then_some((assoc, distance))
                    })
                    .map(|(assoc, distance)| {
                        let color = {
                            let dist_min = *self.otobrite_distance_range.start();
                            let dist_max = *self.otobrite_distance_range.end();
                            let [hue_min, hue_max] = self.otobrite_hue_range;
                            let deg = distance.clamp(dist_min, dist_max) / (dist_max - dist_min)
                                * (hue_max - hue_min)
                                + hue_min;
                            let hue = RgbHue::from_degrees(deg);
                            let hsv = Hsv::new(hue, 1.0, 1.0);
                            let rgb: Srgb = hsv.into_color();
                            let (r, g, b) = rgb.into_components();
                            Scalar::new(b as f64 * 255.0, g as f64 * 255.0, r as f64 * 255.0, 0.0)
                        };

                        let center = {
                            let Point2f { x, y } = assoc.img_point;
                            Point2i::new(x.round() as i32, y.round() as i32)
                        };

                        let xy = [center.x, center.y];
                        (xy, (center, color))
                    })
                    .collect();

                pixels.into_iter().for_each(|(_, (center, color))| {
                    imgproc::circle(
                        &mut canvas,
                        center,
                        1, // radius
                        color,
                        1, // thickness
                        LINE_8,
                        0, // shift
                    )
                    .unwrap();
                });
            }

            // Crop
            let canvas = Mat::roi(&canvas, self.otobrite_image_roi)?;

            // Scale image
            let canvas = {
                let target_size = self.otobrite_fused_present_size as f64;
                let fx = target_size / canvas.cols() as f64;
                let fy = target_size / canvas.rows() as f64;
                let scale = fx.min(fy);

                let mut out = Mat::default();
                imgproc::resize(
                    &canvas,
                    &mut out,
                    Size::default(),
                    scale,
                    scale,
                    INTER_LINEAR,
                )?;
                out
            };

            canvas
        };

        self.otobrite_raw_image = otobrite_raw_image;
        self.otobrite_fused_image = otobrite_fused_image;

        Ok(())
    }

    fn update_kneron(&mut self, msg: msg::KneronMessage) -> Result<()> {
        let msg::KneronMessage { assocs, objects } = msg;
        let orig_image: Mat = make_zero_mat(self.kneron_image_hw);

        let kneron_detection_image = {
            let mut canvas = orig_image.clone();

            // Draw rectangles
            if let Some(objects) = &objects {
                objects.iter().for_each(|object: &msg::Object| {
                    // Sample color using the pointer value of ArcRefC
                    let [r, g, b] = match &object.class_id {
                        Some(class_id) => sample_rgb(class_id),
                        None => [1.0, 1.0, 1.0],
                    };
                    let color = Scalar::new(b, g, r, 0.0);

                    imgproc::rectangle(
                        &mut canvas,
                        object.rect,
                        color,
                        3, // thickness
                        LINE_8,
                        0, // shift
                    )
                    .unwrap();
                });
            }

            // Crop
            let canvas = Mat::roi(&canvas, self.kneron_image_roi)?;

            // Scale image
            let canvas = {
                let target_size = self.kneron_det_present_size as f64;
                let fx = target_size / canvas.cols() as f64;
                let fy = target_size / canvas.rows() as f64;
                let scale = fx.min(fy);

                let mut out = Mat::default();
                imgproc::resize(
                    &canvas,
                    &mut out,
                    Size::default(),
                    scale,
                    scale,
                    INTER_LINEAR,
                )?;
                out
            };

            canvas
        };

        let kneron_fused_image = {
            let mut canvas = orig_image;

            // Draw rectangles
            if let Some(objects) = &objects {
                objects.iter().for_each(|object: &msg::Object| {
                    // Sample color using the pointer value of ArcRefC
                    let [r, g, b] = match &object.class_id {
                        Some(class_id) => sample_rgb(class_id),
                        None => [1.0, 1.0, 1.0],
                    };
                    let color = Scalar::new(b, g, r, 0.0);

                    imgproc::rectangle(
                        &mut canvas,
                        object.rect,
                        color,
                        1, // thickness
                        LINE_8,
                        0, // shift
                    )
                    .unwrap();
                });
            }

            // Draw points
            if let Some(assocs) = assocs {
                let pixels: HashMap<_, _> = assocs
                    .par_iter()
                    .filter_map(|assoc| {
                        let distance =
                            na::distance(&na::Point3::origin(), &assoc.pcd_point.position);
                        self.kneron_distance_range
                            .contains(&distance)
                            .then_some((assoc, distance))
                    })
                    .map(|(assoc, _distance)| {
                        let color = {
                            let [r, g, b] = match assoc.object.as_deref() {
                                Some(msg::Object {
                                    class_id: Some(ref class_id),
                                    ..
                                }) => sample_rgb(class_id),
                                _ => [0.5, 0.5, 0.5],
                            };
                            Scalar::new(b, g, r, 0.0)
                        };
                        let center = {
                            let Point2f { x, y } = assoc.img_point;
                            Point2i::new(x.round() as i32, y.round() as i32)
                        };

                        let xy = [center.x, center.y];
                        (xy, (center, color))
                    })
                    .collect();

                pixels.into_iter().for_each(|(_, (center, color))| {
                    imgproc::circle(
                        &mut canvas,
                        center,
                        1, // radius
                        color,
                        1, // thickness
                        LINE_8,
                        0, // shift
                    )
                    .unwrap();
                });
            }

            // Crop
            let canvas = Mat::roi(&canvas, self.kneron_image_roi)?;

            // Scale image
            let canvas = {
                let target_size = self.kneron_fused_present_size as f64;
                let fx = target_size / canvas.cols() as f64;
                let fy = target_size / canvas.rows() as f64;
                let scale = fx.min(fy);

                let mut out = Mat::default();
                imgproc::resize(
                    &canvas,
                    &mut out,
                    Size::default(),
                    scale,
                    scale,
                    INTER_LINEAR,
                )?;
                out
            };

            canvas
        };

        self.kneron_det_image = kneron_detection_image;
        self.kneron_fused_image = kneron_fused_image;

        Ok(())
    }
}

fn make_zero_mat([h, w]: [usize; 2]) -> Mat {
    Mat::zeros(h as i32, w as i32, CV_32FC3)
        .unwrap()
        .to_mat()
        .unwrap()
}
