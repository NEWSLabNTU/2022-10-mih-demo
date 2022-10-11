mod config;
mod kiss3d_gui;
// mod rate_meter;

use crate::config::Config;
use anyhow::Result;
use async_std::task::spawn_blocking;
use clap::Parser;
use futures::{future, future::FutureExt as _, stream::StreamExt as _};
use r2r::{
    autoware_auto_perception_msgs::msg::DetectedObjects, sensor_msgs::msg::PointCloud2, Context,
    Node, QosProfile,
};
use serde_loader::Json5Path;
use std::{path::PathBuf, time::Duration};

#[derive(Parser)]
struct Opts {
    #[clap(long)]
    pub config: PathBuf,
}

#[async_std::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();
    let config: Config = Json5Path::open_and_take(&opts.config)?;
    let Config {
        namespace,
        pcd_topic,
        det_topic,
        ..
    } = config;

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, env!("CARGO_PKG_NAME"), &namespace)?;

    let pcd_sub = node.subscribe::<PointCloud2>(&pcd_topic, QosProfile::default())?;
    let det_sub = node.subscribe::<DetectedObjects>(&det_topic, QosProfile::default())?;

    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    let (gui3d_future, gui3d_tx) = kiss3d_gui::start();

    let pcd_forward = pcd_sub
        .map(kiss3d_gui::Message::from)
        .map(Ok)
        .forward(gui3d_tx.clone().into_sink());
    let det_forward = det_sub
        .map(kiss3d_gui::Message::from)
        .map(Ok)
        .forward(gui3d_tx.into_sink());

    let join1 = future::try_join(gui3d_future, spin_future.map(Ok));
    let join2 = future::try_join(
        pcd_forward.map(|result| result.map_err(|_| ())),
        det_forward.map(|result| result.map_err(|_| ())),
    );

    futures::select! {
        result = join1.fuse() => {
            result?;
        }
        _ = join2.fuse() => {}
    };

    Ok(())
}
