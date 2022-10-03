use anyhow::{ensure, Result};
use async_std::task::{spawn_blocking, JoinHandle};
use flume::RecvTimeoutError;
use opencv::{
    core::{Rect, Scalar, Vec3b, VecN, CV_8UC3},
    highgui,
    prelude::*,
};
use r2r::{
    geometry_msgs::msg::Pose2D,
    log_error,
    sensor_msgs::msg::Image,
    vision_msgs::msg::{BoundingBox2D, Detection2DArray},
};
use std::time::{Duration, Instant};

const INTERVAL: Duration = Duration::from_millis(100);

pub fn start() -> (JoinHandle<Result<()>>, flume::Sender<Message>) {
    let (tx, rx) = flume::bounded(2);

    let handle = spawn_blocking(move || {
        let mut state = State::default();
        let mut until = Instant::now() + INTERVAL;

        loop {
            match rx.recv_deadline(until) {
                Ok(msg) => {
                    state.update(msg);
                    if Instant::now() < until {
                        continue;
                    }
                }
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => {}
            }

            state.step()?;
            until = Instant::now() + INTERVAL;
        }

        anyhow::Ok(())
    });

    (handle, tx)
}

#[derive(Default)]
struct State {
    image: Option<Mat>,
    rects: Vec<Rect>,
}

impl State {
    fn step(&mut self) -> Result<()> {
        if let Some(image) = &self.image {
            highgui::imshow("demo", image)?;
            let _key = highgui::wait_key(1)?;
        }

        Ok(())
    }

    fn update(&mut self, msg: Message) {
        match msg {
            Message::Image(image) => {
                self.update_image(image);
            }
            Message::BBox(bbox) => {
                self.update_det(bbox);
            }
        }
    }

    fn update_image(&mut self, image: Image) {
        let mat = match image2mat(&image) {
            Ok(mat) => mat,
            Err(err) => {
                log_error!(env!("CARGO_PKG_NAME"), "invalid input image: {}", err);
                return;
            }
        };
        self.image = Some(mat);
    }

    fn update_det(&mut self, array: Detection2DArray) {
        self.rects = array
            .detections
            .iter()
            .map(|det| {
                let BoundingBox2D {
                    size_x,
                    size_y,
                    center: Pose2D { x: cx, y: cy, .. },
                } = det.bbox;

                // left-top x and y
                let ltx = cx - size_x / 2.0;
                let lty = cy - size_y / 2.0;

                Rect {
                    x: ltx as i32,
                    y: lty as i32,
                    width: size_x as i32,
                    height: size_y as i32,
                }
            })
            .collect();
    }
}

fn image2mat(image: &Image) -> Result<Mat> {
    let Image {
        height,
        width,
        ref encoding,
        is_bigendian,
        step,
        ref data,
        ..
    } = *image;

    ensure!(encoding == "bgr8");
    ensure!(is_bigendian == 0);
    ensure!(step == width * 3);
    ensure!(data.len() == step as usize * height as usize);

    let mut mat =
        Mat::new_rows_cols_with_default(height as i32, width as i32, CV_8UC3, Scalar::all(0.0))?;

    data.chunks_exact(3).enumerate().for_each(|(pidx, bytes)| {
        let col = pidx % width as usize;
        let row = pidx / width as usize;
        let pixel: &mut Vec3b = mat.at_2d_mut(row as i32, col as i32).unwrap();
        let bytes: [u8; 3] = bytes.try_into().unwrap();
        *pixel = VecN(bytes);
    });

    Ok(mat)
}

pub enum Message {
    Image(Image),
    BBox(Detection2DArray),
}

impl From<Detection2DArray> for Message {
    fn from(v: Detection2DArray) -> Self {
        Self::BBox(v)
    }
}

impl From<Image> for Message {
    fn from(v: Image) -> Self {
        Self::Image(v)
    }
}
