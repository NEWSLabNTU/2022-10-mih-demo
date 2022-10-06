mod config;
mod fuse;
mod kiss3d_gui;
mod message;
mod opencv_gui;
mod rate_meter;
mod yaml_loader;

use crate::{config::Config, message as msg, rate_meter::RateMeter};
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
    } = &config;

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "demo_viz", namespace)?;

    // Init rate meters
    let pcd_meter = Arc::new(RateMeter::new_secs());
    let det_meter = Arc::new(RateMeter::new_secs());
    let img_meter = Arc::new(RateMeter::new_secs());

    let rate_printing_future = {
        let pcd_meter = pcd_meter.clone();
        let det_meter = det_meter.clone();
        let img_meter = img_meter.clone();

        async move {
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

            futures::join!(pcd_rate_print, det_rate_print, img_rate_print);
        }
    };

    // Initialize subscriptions
    let pcd_sub = node.subscribe::<PointCloud2>(pcd_topic, QosProfile::default())?;
    let det_sub = node.subscribe::<Detection2DArray>(det_topic, QosProfile::default())?;
    let img_sub = node.subscribe::<Image>(img_topic, QosProfile::default())?;

    // Merge subscription streams into one
    let input_stream = {
        let pcd_stream = pcd_sub
            .inspect(|_| {
                pcd_meter.bump();
            })
            .map(msg::InputMessage::from)
            .boxed();
        let det_stream = det_sub
            .inspect(|_| {
                det_meter.bump();
            })
            .map(msg::InputMessage::from)
            .boxed();
        let img_stream = img_sub
            .inspect(|_| {
                img_meter.bump();
            })
            .map(msg::InputMessage::from)
            .boxed();
        futures::stream::select_all([pcd_stream, det_stream, img_stream])
    };

    // Start image/pcd fusing worker
    let fuse_stream = fuse::start(input_stream, &config)?;

    // Split fuse worker output into two parts
    let (split_future, opencv_rx, kiss3d_rx) = split(fuse_stream.boxed());

    // Start OpenCV GUI
    let opencv_future = opencv_gui::start(opencv_rx.into_stream());

    // Start Kiss3d GUI
    let kiss3d_future = kiss3d_gui::start(kiss3d_rx.into_stream());

    // Spin the ROS node
    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    // Join all futures
    let join1 = future::join4(
        split_future,
        kiss3d_future,
        spin_future,
        rate_printing_future,
    );
    let join2 = future::try_join(join1.map(|_| anyhow::Ok(())), opencv_future);
    join2.await?;

    Ok(())
}

fn split(
    mut stream: impl Stream<Item = msg::FuseMessage> + Unpin + Send,
) -> (
    impl Future<Output = ()> + Send,
    flume::Receiver<msg::OpencvGuiMessage>,
    flume::Receiver<msg::Kiss3dMessage>,
) {
    let (opencv_tx, opencv_rx) = flume::bounded(2);
    let (kiss3d_tx, kiss3d_rx) = flume::bounded(2);

    let future = async move {
        while let Some(in_msg) = stream.next().await {
            let msg::FuseMessage {
                kiss3d_msg,
                opencv_msg,
            } = in_msg;

            let opencv_send = async {
                if let Some(msg) = opencv_msg {
                    opencv_tx.send_async(msg).await.is_ok()
                } else {
                    true
                }
            };
            let kiss3d_send = async {
                if let Some(msg) = kiss3d_msg {
                    kiss3d_tx.send_async(msg).await.is_ok()
                } else {
                    true
                }
            };

            let (opencv_ok, kiss3d_ok) = futures::join!(opencv_send, kiss3d_send);

            if !(opencv_ok && kiss3d_ok) {
                break;
            }
        }
    };

    (future, opencv_rx, kiss3d_rx)
}
