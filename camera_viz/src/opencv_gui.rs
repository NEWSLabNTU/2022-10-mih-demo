use crate::{color_sampling::sample_rgb, config::Config, message as msg};
use anyhow::Result;
use async_std::task::spawn_blocking;
use futures::prelude::*;
use opencv::{
    core::{Point2f, Point2i, Scalar, CV_32FC3},
    highgui, imgproc,
    prelude::*,
};
use std::{
    num::NonZeroUsize,
    time::{Duration, Instant},
};

const INTERVAL: Duration = Duration::from_millis(100);

pub async fn start(
    config: &Config,
    stream: impl Stream<Item = msg::OpencvMessage> + Unpin + Send,
) -> Result<()> {
    let Config {
        otobrite_image_hw,
        kneron_image_hw,
        ..
    } = *config;

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
                kneron_image_hw,
            }
        };
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

struct State {
    kneron_image: Mat,
    otobrite_image: Mat,
    otobrite_image_hw: [usize; 2],
    kneron_image_hw: [usize; 2],
}

impl State {
    fn step(&mut self) -> Result<()> {
        highgui::imshow("Kneron Camera", &self.kneron_image)?;
        highgui::imshow("Otobrite Camera", &self.otobrite_image)?;
        let _key = highgui::wait_key(1)?;

        Ok(())
    }

    fn update(&mut self, msg: msg::OpencvMessage) {
        use msg::OpencvMessage as M;

        match msg {
            M::Otobrite(msg) => self.update_otobrite(msg),
            M::Kneron(msg) => self.update_kneron(msg),
        }
    }

    fn update_otobrite(&mut self, msg: msg::OtobriteMessage) {
        let msg::OtobriteMessage { image, assocs } = msg;

        let mut canvas: Mat = image.unwrap_or_else(|| make_zero_mat(self.otobrite_image_hw));

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

        self.otobrite_image = canvas;
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
