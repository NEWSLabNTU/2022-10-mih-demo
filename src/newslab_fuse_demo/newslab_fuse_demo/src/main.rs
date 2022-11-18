mod color_sampling;
mod config;
mod fuse;
mod kiss3d_gui;
mod message;
mod opencv_gui;
mod point_projection;
// mod rect_rtree;
mod yaml_loader;
// mod rate_meter;

use crate::{config::Config, message as msg};
use anyhow::Result;
use async_std::task::spawn_blocking;
use clap::Parser;
use futures::{future, prelude::*};
use r2r::{
    log_info,
    sensor_msgs::msg::{Image, PointCloud2},
    vision_msgs::msg::Detection2DArray,
    Context, Node, QosProfile,
};
use serde_loader::Json5Path;
use std::{path::PathBuf, time::Duration};

/// The type defines the program arguments.
#[derive(Parser)]
struct Opts {
    #[clap(long)]
    pub config: PathBuf,
}

#[async_std::main]
async fn main() -> Result<()> {
    // Parse program arguments.
    let opts = Opts::parse();

    // Load the configuration file.
    let config: Config = Json5Path::open_and_take(&opts.config)?;
    let Config {
        namespace,
        pcd_topic,
        otobrite_img_topic,
        kneron_det_topic,
        ..
    } = &config;

    // Create a ROS node.
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, env!("CARGO_PKG_NAME"), namespace)?;

    // Create ROS subscriptions
    log_info!(
        env!("CARGO_PKG_NAME"),
        "Subscribe point cloud from {}",
        pcd_topic
    );
    let pcd_sub = node.subscribe::<PointCloud2>(pcd_topic, QosProfile::default())?;

    log_info!(
        env!("CARGO_PKG_NAME"),
        "Subscribe Otobrite camera image from {}",
        otobrite_img_topic
    );
    let otobrite_img_sub = node.subscribe::<Image>(otobrite_img_topic, QosProfile::default())?;

    log_info!(
        env!("CARGO_PKG_NAME"),
        "Subscribe Kneron camera detection from {}",
        kneron_det_topic
    );
    let kneron_det_sub =
        node.subscribe::<Detection2DArray>(kneron_det_topic, QosProfile::default())?;

    // Merge subscription streams into one stream using `select` operation
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
    let kiss3d_future = kiss3d_gui::start(&config, kiss3d_rx.into_stream());

    // Create a future to spin the ROS node
    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    // Join all futures
    let join1 = future::join3(split_future, kiss3d_future, spin_future);
    let join2 = future::try_join(join1.map(|_| anyhow::Ok(())), opencv_future);
    join2.await?;

    Ok(())
}

/// Splits a stream and forward the messages to two respective channels.
///
/// # Returns
/// It returns a tuple (future, opencv_rx, kiss3d_rx).
/// - `future` is polled to forward the messages in the stream to the senders.
/// - `opencv_rx` is receiver of the channel that collecting OpenCV messages.
/// - `kiss3d_rx` is receiver of the channel that collecting Kiss3d messages.
fn split(
    mut stream: impl Stream<Item = msg::FuseMessage> + Unpin + Send,
) -> (
    impl Future<Output = ()> + Send,
    flume::Receiver<msg::OpencvMessage>,
    flume::Receiver<msg::Kiss3dMessage>,
) {
    // Create two channels
    let (opencv_tx, opencv_rx) = flume::bounded(2);
    let (kiss3d_tx, kiss3d_rx) = flume::bounded(2);

    // Create a future that forwards stream messages to respective channels.
    let future = async move {
        while let Some(in_msg) = stream.next().await {
            use msg::FuseMessage as M;

            let ok: bool = match in_msg {
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
