mod config;
mod kiss3d_gui;
mod opencv_gui;
mod rate_meter;

use crate::{config::Config, rate_meter::RateMeter};
use anyhow::Result;
use async_std::task::spawn_blocking;
use clap::Parser;
use futures::{future, future::FutureExt as _, stream::StreamExt as _};
use r2r::{
    log_info,
    sensor_msgs::msg::{Image, PointCloud2},
    vision_msgs::msg::Detection2DArray,
    Context, Node, QosProfile,
};
use serde_loader::Json5Path;
use std::{path::PathBuf, sync::Arc, time::Duration};

#[derive(Parser)]
struct Opts {
    pub config: PathBuf,
}

#[async_std::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();
    let config: Config = Json5Path::open_and_take(&opts.config)?;
    let Config {
        namespace,
        pcd_topic,
        img_topic,
        det_topic,
        ..
    } = config;

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "demo_viz", &namespace)?;

    let pcd_sub = node.subscribe::<PointCloud2>(&pcd_topic, QosProfile::default())?;
    let det_sub = node.subscribe::<Detection2DArray>(&det_topic, QosProfile::default())?;
    let img_sub = node.subscribe::<Image>(&img_topic, QosProfile::default())?;

    let pcd_meter = Arc::new(RateMeter::new_secs());
    let det_meter = Arc::new(RateMeter::new_secs());
    let img_meter = Arc::new(RateMeter::new_secs());

    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    let (gui2d_future, gui2d_tx) = opencv_gui::start();
    let (gui3d_future, gui3d_tx) = kiss3d_gui::start();

    let pcd_forward = pcd_sub
        .inspect(|_| {
            pcd_meter.bump();
        })
        .map(kiss3d_gui::Message::from)
        .map(Ok)
        .forward(gui3d_tx.clone().into_sink());
    let det_forward = det_sub
        .inspect(|_| {
            det_meter.bump();
        })
        .map(opencv_gui::Message::from)
        .map(Ok)
        .forward(gui2d_tx.clone().into_sink());
    let img_forward = img_sub
        .inspect(|_| {
            img_meter.bump();
        })
        .map(opencv_gui::Message::from)
        .map(Ok)
        .forward(gui2d_tx.into_sink());

    let pcd_rate_print = pcd_meter.rate_stream().for_each(|rate| {
        let topic = &pcd_topic;
        async move {
            log_info!(env!("CARGO_PKG_NAME"), "{} rate {} msgs/s", topic, rate);
        }
    });
    let det_rate_print = det_meter.rate_stream().for_each(|rate| {
        let topic = &det_topic;
        async move {
            log_info!(env!("CARGO_PKG_NAME"), "{} rate {} msgs/s", topic, rate);
        }
    });
    let img_rate_print = img_meter.rate_stream().for_each(|rate| {
        let topic = &img_topic;
        async move {
            log_info!(env!("CARGO_PKG_NAME"), "{} rate {} msgs/s", topic, rate);
        }
    });

    let join1 = future::try_join3(gui2d_future, gui3d_future, spin_future.map(Ok));
    let join2 = future::try_join3(
        pcd_forward.map(|result| result.map_err(|_| ())),
        det_forward.map(|result| result.map_err(|_| ())),
        img_forward.map(|result| result.map_err(|_| ())),
    );
    let join3 = future::join3(pcd_rate_print, det_rate_print, img_rate_print);

    futures::select! {
        result = join1.fuse() => {
            result?;
        }
        _ = join2.fuse() => {}
        _ = join3.fuse() => {}
    };

    Ok(())
}
