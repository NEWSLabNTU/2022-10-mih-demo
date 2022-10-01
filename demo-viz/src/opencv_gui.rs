use anyhow::Result;
use async_std::{
    stream::interval,
    task::{spawn, JoinHandle},
};
use futures::{
    future::Either::*,
    stream,
    stream::{StreamExt as _, TryStreamExt as _},
};
use opencv::{core::Rect, highgui, prelude::*};
use r2r::{
    geometry_msgs::msg::Pose2D,
    sensor_msgs::msg::Image,
    vision_msgs::msg::{BoundingBox2D, BoundingBox2DArray},
};
use std::time::Duration;

const INTERVAL: Duration = Duration::from_millis(100);

pub fn start() -> (JoinHandle<Result<()>>, flume::Sender<Message>) {
    let (tx, rx) = flume::bounded(2);

    let handle = spawn(async move {
        let int_stream = interval(INTERVAL).map(Left);
        let image_stream = rx.into_stream().map(Right);
        let stream = stream::select(int_stream, image_stream);
        let init_state = State::default();

        stream
            .map(anyhow::Ok)
            .try_fold(init_state, |mut state, either| async move {
                match either {
                    // update gui
                    Left(()) => {
                        state.step()?;
                    }
                    // receive image
                    Right(msg) => {
                        state.update(msg)?;
                    }
                }

                anyhow::Ok(state)
            })
            .await?;

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
        }

        Ok(())
    }

    fn update(&mut self, msg: Message) -> Result<()> {
        match msg {
            Message::Image(image) => {
                self.update_image(image)?;
            }
            Message::BBox(bbox) => {
                self.update_det(bbox)?;
            }
        }

        Ok(())
    }

    fn update_image(&mut self, image: Image) -> Result<()> {
        let mat = image2mat(&image);
        self.image = Some(mat);
        Ok(())
    }

    fn update_det(&mut self, array: BoundingBox2DArray) -> Result<()> {
        self.rects = array
            .boxes
            .iter()
            .map(|bbox| {
                let BoundingBox2D {
                    size_x,
                    size_y,
                    center: Pose2D { x: cx, y: cy, .. },
                } = *bbox;

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

        Ok(())
    }
}

fn image2mat(image: &Image) -> Mat {
    let Image {
        height,
        width,
        ref encoding,
        is_bigendian,
        step,
        ref data,
        ..
    } = *image;

    todo!();
}

pub enum Message {
    Image(Image),
    BBox(BoundingBox2DArray),
}

impl From<BoundingBox2DArray> for Message {
    fn from(v: BoundingBox2DArray) -> Self {
        Self::BBox(v)
    }
}

impl From<Image> for Message {
    fn from(v: Image) -> Self {
        Self::Image(v)
    }
}
