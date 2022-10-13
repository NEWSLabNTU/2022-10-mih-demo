use crate::{color_sampling::sample_rgb, config::Config, message as msg};
use anyhow::Result;
use async_std::task::spawn_blocking;
use futures::prelude::*;
use nalgebra as na;
use opencv::{
    calib3d,
    core::{no_array, Point2f, Point2i, Scalar, CV_32FC3},
    highgui, imgproc,
    prelude::*,
};
use palette::{Hsv, IntoColor, RgbHue, Srgb};
use std::{
    num::NonZeroUsize,
    time::{Duration, Instant},
};

const MAX_DISTANCE: f32 = 5.0;
const INTERVAL: Duration = Duration::from_millis(100);

pub async fn start(
    config: &Config,
    stream: impl Stream<Item = msg::OpencvMessage> + Unpin + Send,
) -> Result<()> {
    let Config {
        otobrite_image_hw,
        kneron_image_hw,
        ref otobrite_intrinsics_file,
        ref otobrite_extrinsics_file,
        ..
    } = *config;
    let otobrite_camera_matrix = otobrite_intrinsics_file.camera_matrix.to_opencv();
    let otobrite_dist_coefs = otobrite_intrinsics_file.distortion_coefficients.to_opencv();
    let otobrite_pose: na::Isometry3<f32> = na::convert_ref(&**otobrite_extrinsics_file);

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
                otobrite_image: make_zero_mat(otobrite_image_hw),
                otobrite_image_hw,
                otobrite_camera_matrix,
                otobrite_dist_coefs,
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
    otobrite_image: Mat,
    otobrite_image_hw: [usize; 2],
    otobrite_pose: na::Isometry3<f32>,
    otobrite_camera_matrix: Mat,
    otobrite_dist_coefs: Mat,
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
            M::Kneron(msg) => self.update_kneron(msg),
        }

        Ok(())
    }

    fn update_otobrite(&mut self, msg: msg::OtobriteMessage) -> Result<()> {
        let msg::OtobriteMessage { image, assocs } = msg;

        let mut canvas = match image {
            Some(image) => {
                let mut out = Mat::default();
                calib3d::undistort(
                    &image,
                    &mut out,
                    &self.otobrite_camera_matrix,
                    &self.otobrite_dist_coefs,
                    &no_array(),
                )?;
                out
            }
            None => make_zero_mat(self.otobrite_image_hw),
        };

        // let mut canvas: Mat = image.unwrap_or_else(|| make_zero_mat(self.otobrite_image_hw));

        // Draw points
        if let Some(assocs) = assocs {
            assocs
                .iter()
                .filter(|assoc| {
                    let camera_point = &self.otobrite_pose * assoc.pcd_point.position;
                    camera_point.z > 0.0
                })
                .for_each(|assoc| {
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

        self.otobrite_image = canvas;

        Ok(())
    }

    fn update_kneron(&mut self, msg: msg::KneronMessage) {
        let msg::KneronMessage { assocs, rects } = msg;
        let mut canvas: Mat = make_zero_mat(self.kneron_image_hw);

        // Draw rectangles
        if let Some(rects) = rects {
            rects.clone().flatten().for_each(|rect: msg::ArcRect| {
                // Sample color using the pointer value of ArcRefC
                let [r, g, b] = sample_rgb(&rect);
                let color = Scalar::new(b, g, r, 0.0);

                imgproc::rectangle(
                    &mut canvas,
                    *rect,
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
                    let [r, g, b] = if let Some(rect) = &assoc.rect {
                        sample_rgb(rect)
                    } else {
                        [0.1, 0.1, 0.1]
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

        self.kneron_image = canvas;
    }
}

fn make_zero_mat([h, w]: [usize; 2]) -> Mat {
    Mat::zeros(h as i32, w as i32, CV_32FC3)
        .unwrap()
        .to_mat()
        .unwrap()
}
