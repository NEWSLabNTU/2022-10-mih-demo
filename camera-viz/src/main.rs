mod color_sampling;
mod config;
mod fuse;
mod kiss3d_gui;
mod message;
mod opencv_gui;
mod rect_rtree;
mod yaml_loader;
// mod rate_meter;

use crate::{config::Config, message as msg};
use anyhow::Result;
use async_std::task::spawn_blocking;
use clap::Parser;
use futures::{future, prelude::*};
use r2r::{
    sensor_msgs::msg::{Image, PointCloud2},
    vision_msgs::msg::Detection2DArray,
    Context, Node, QosProfile,
};
use serde_loader::Json5Path;
use std::{path::PathBuf, time::Duration};

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
        otobrite_img_topic,
        kneron_det_topic,
        ..
    } = &config;

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "demo_viz", namespace)?;

    // Initialize subscriptions
    let pcd_sub = node.subscribe::<PointCloud2>(pcd_topic, QosProfile::default())?;
    let otobrite_img_sub = node.subscribe::<Image>(otobrite_img_topic, QosProfile::default())?;
    let kneron_det_sub =
        node.subscribe::<Detection2DArray>(kneron_det_topic, QosProfile::default())?;

    // Merge subscription streams into one
    let input_stream = {
        let pcd_stream = pcd_sub.map(msg::InputMessage::PointCloud2).boxed();
        let kneron_det_stream = kneron_det_sub.map(msg::InputMessage::BBox).boxed();
        let otobrite_img_stream = otobrite_img_sub
            .map(msg::InputMessage::OtobriteImage)
            .boxed();
        futures::stream::select_all([pcd_stream, kneron_det_stream, otobrite_img_stream])
    };

    // Start image/pcd fusing worker
    let fuse_stream = fuse::start(input_stream, &config)?;

    // Split fuse worker output into two parts
    let (split_future, opencv_rx, kiss3d_rx) = split(fuse_stream.boxed());

    // Start OpenCV GUI
    let opencv_future = opencv_gui::start(&config, opencv_rx.into_stream());

    // Start Kiss3d GUI
    let kiss3d_future = kiss3d_gui::start(kiss3d_rx.into_stream());

    // Spin the ROS node
    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    // Join all futures
    let join1 = future::join3(split_future, kiss3d_future, spin_future);
    let join2 = future::try_join(join1.map(|_| anyhow::Ok(())), opencv_future);
    join2.await?;

    Ok(())
}

fn split(
    mut stream: impl Stream<Item = msg::FuseMessage> + Unpin + Send,
) -> (
    impl Future<Output = ()> + Send,
    flume::Receiver<msg::OpencvMessage>,
    flume::Receiver<msg::Kiss3dMessage>,
) {
    let (opencv_tx, opencv_rx) = flume::bounded(2);
    let (kiss3d_tx, kiss3d_rx) = flume::bounded(2);

    let future = async move {
        while let Some(in_msg) = stream.next().await {
            use msg::FuseMessage as M;

            let ok = match in_msg {
                M::Otobrite(msg) => opencv_tx.send_async(msg.into()).await.is_ok(),
                M::Kneron(msg) => opencv_tx.send_async(msg.into()).await.is_ok(),
                M::Kiss3d(msg) => kiss3d_tx.send_async(msg).await.is_ok(),
            };

            if !ok {
                break;
            }
        }
    };

    (future, opencv_rx, kiss3d_rx)
}
