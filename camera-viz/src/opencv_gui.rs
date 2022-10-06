use crate::{message as msg, utils::sample_rgb};
use anyhow::Result;
use async_std::task::spawn_blocking;
use futures::prelude::*;
use opencv::{
    core::{Point2f, Point2i, Scalar},
    highgui, imgproc,
    prelude::*,
};
use std::time::{Duration, Instant};

const INTERVAL: Duration = Duration::from_millis(100);

pub async fn start(stream: impl Stream<Item = msg::OpencvGuiMessage> + Unpin + Send) -> Result<()> {
    let (tx, rx) = flume::bounded(2);

    let forward_future = stream.map(Ok).forward(tx.into_sink()).map(|_result| ());
    let handle_future = spawn_blocking(move || {
        use flume::RecvTimeoutError as E;

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

#[derive(Default)]
struct State {
    image: Option<Mat>,
}

impl State {
    fn step(&mut self) -> Result<()> {
        if let Some(image) = &self.image {
            highgui::imshow("demo", image)?;
            let _key = highgui::wait_key(1)?;
        }

        Ok(())
    }

    fn update(&mut self, msg: msg::OpencvGuiMessage) {
        let msg::OpencvGuiMessage {
            mut image,
            rects,
            assocs,
        } = msg;

        // Draw rectangles
        rects.clone().flatten().for_each(|rect: msg::ArcRect| {
            // Sample color using the pointer value of ArcRefC
            let [r, g, b] = sample_rgb(&rect);
            let color = Scalar::new(b, g, r, 0.0);

            imgproc::rectangle(
                &mut image,
                *rect,
                color,
                1, // thickness
                imgproc::LINE_8,
                0, // shift
            )
            .unwrap();
        });

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
                    &mut image,
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

        self.image = Some(image);
    }
}
