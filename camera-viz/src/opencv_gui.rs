use crate::message as msg;
use anyhow::Result;
use async_std::task::spawn_blocking;
use futures::prelude::*;
use opencv::{core::Rect, highgui, prelude::*};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

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
    rects: Option<Arc<Vec<Rect>>>,
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
        let msg::OpencvGuiMessage { image, rects } = msg;
        self.image = Some(image);
        self.rects = Some(rects);
        todo!();
    }
}
