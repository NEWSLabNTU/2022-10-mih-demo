use crate::{color_sampling::sample_rgb, config::Config, message as msg};
use anyhow::Result;
use async_std::task::spawn_blocking;
use futures::prelude::*;
use nalgebra as na;
use opencv::{
    core::{Point2f, Point2i, Scalar, Size, CV_32FC3},
    highgui,
    imgproc::{self, INTER_LINEAR},
    prelude::*,
};
use palette::{Hsv, IntoColor, RgbHue, Srgb};
use std::{
    num::NonZeroUsize,
    time::{Duration, Instant},
};

const MAX_DISTANCE: f32 = 10.0;
const INTERVAL: Duration = Duration::from_millis(100);

pub async fn start(
    config: &Config,
    stream: impl Stream<Item = msg::OpencvMessage> + Unpin + Send,
) -> Result<()> {
    let Config {
        otobrite_image_hw,
        kneron_image_hw,
        ref otobrite_intrinsics_file,
        otobrite_present_size,
        kneron_present_size,
        ..
    } = *config;
    let otobrite_camera_matrix = otobrite_intrinsics_file.camera_matrix.to_opencv();
    let otobrite_dist_coefs = otobrite_intrinsics_file.distortion_coefficients.to_opencv();
    let otobrite_pose: na::Isometry3<f32> = na::convert(config.otobrite_pose());

    let (tx, rx) = flume::bounded(2);

    let forward_future = stream.map(Ok).forward(tx.into_sink()).map(|_result| ());
    let handle_future = spawn_blocking(move || {
        use flume::RecvTimeoutError as E;

        let mut state = {
            let convert_hw = |[h, w]: [NonZeroUsize; 2]| [h.get(), w.get()];
            let otobrite_image_hw = convert_hw(otobrite_image_hw);
            let kneron_image_hw = convert_hw(kneron_image_hw);

            State {
                kneron_image: make_zero_mat(kneron_image_hw),
                kneron_present_size,
                otobrite_image: make_zero_mat(otobrite_image_hw),
                otobrite_image_hw,
                otobrite_camera_matrix,
                otobrite_dist_coefs,
                otobrite_present_size,
                otobrite_pose,
                kneron_image_hw,
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
            until = Instant::now() + INTERVAL;
        }

        anyhow::Ok(())
    });

    futures::try_join!(forward_future.map(|()| anyhow::Ok(())), handle_future)?;
    Ok(())
}

struct State {
    kneron_image: Mat,
    kneron_present_size: usize,
    otobrite_image: Mat,
    otobrite_image_hw: [usize; 2],
    otobrite_pose: na::Isometry3<f32>,
    otobrite_camera_matrix: Mat,
    otobrite_dist_coefs: Mat,
    otobrite_present_size: usize,
    kneron_image_hw: [usize; 2],
}

impl State {
    fn step(&mut self) -> Result<()> {
        highgui::imshow("Kneron Camera", &self.kneron_image)?;
        highgui::imshow("Otobrite Camera", &self.otobrite_image)?;
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

        // let mut canvas = match image {
        //     Some(image) => {
        //         let mut out = Mat::default();
        //         calib3d::undistort(
        //             &image,
        //             &mut out,
        //             &self.otobrite_camera_matrix,
        //             &self.otobrite_dist_coefs,
        //             &no_array(),
        //         )?;
        //         out
        //     }
        //     None => make_zero_mat(self.otobrite_image_hw),
        // };

        let mut canvas: Mat = image.unwrap_or_else(|| make_zero_mat(self.otobrite_image_hw));

        // Draw points
        if let Some(assocs) = assocs {
            assocs.iter().for_each(|assoc| {
                let distance = na::distance(&na::Point3::origin(), &assoc.pcd_point.position);
                let color = {
                    let deg = distance.clamp(0.0, MAX_DISTANCE) / MAX_DISTANCE * 270.0;
                    let hue = RgbHue::from_degrees(deg);
                    let hsv = Hsv::new(hue, 0.8, 1.0);
                    let rgb: Srgb = hsv.into_color();
                    let (r, g, b) = rgb.into_components();
                    Scalar::new(b as f64 * 255.0, g as f64 * 255.0, r as f64 * 255.0, 0.0)
                };

                // let color = {
                //     let [r, g, b] = if let Some(rect) = &assoc.rect {
                //         sample_rgb(rect)
                //     } else {
                //         [0.1, 0.1, 0.1]
                //     };
                //     Scalar::new(b, g, r, 0.0)
                // };
                let center = {
                    let Point2f { x, y } = assoc.img_point;
                    Point2i::new(x.round() as i32, y.round() as i32)
                };

                imgproc::circle(
                    &mut canvas,
                    center,
                    1, // radius
                    color,
                    1, // thickness
                    imgproc::LINE_8,
                    0, // shift
                )
                .unwrap();
            });
        }

        // Scale image
        let canvas = {
            let target_size = self.otobrite_present_size as f64;
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

        self.otobrite_image = canvas;

        Ok(())
    }

    fn update_kneron(&mut self, msg: msg::KneronMessage) -> Result<()> {
        let msg::KneronMessage { assocs, objects } = msg;
        let mut canvas: Mat = make_zero_mat(self.kneron_image_hw);

        // Draw rectangles
        if let Some(objects) = objects {
            objects.clone().flatten().for_each(|object: msg::ArcObj| {
                // Sample color using the pointer value of ArcRefC
                let [r, g, b] = sample_rgb(&object.class_id);
                let color = Scalar::new(b, g, r, 0.0);

                imgproc::rectangle(
                    &mut canvas,
                    object.rect,
                    color,
                    1, // thickness
                    imgproc::LINE_8,
                    0, // shift
                )
                .unwrap();
            });
        }

        // Draw points
        if let Some(assocs) = assocs {
            assocs.iter().for_each(|assoc| {
                let color = {
                    let [r, g, b] = match &assoc.object {
                        Some(object) => sample_rgb(&object.class_id),
                        None => [0.5, 0.5, 0.5],
                    };
                    Scalar::new(b, g, r, 0.0)
                };
                let center = {
                    let Point2f { x, y } = assoc.img_point;
                    Point2i::new(x.round() as i32, y.round() as i32)
                };

                imgproc::circle(
                    &mut canvas,
                    center,
                    1, // radius
                    color,
                    1, // thickness
                    imgproc::LINE_8,
                    0, // shift
                )
                .unwrap();
            });
        }

        // Scale image
        let canvas = {
            let target_size = self.kneron_present_size as f64;
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

        self.kneron_image = canvas;

        Ok(())
    }
}

fn make_zero_mat([h, w]: [usize; 2]) -> Mat {
    Mat::zeros(h as i32, w as i32, CV_32FC3)
        .unwrap()
        .to_mat()
        .unwrap()
}
