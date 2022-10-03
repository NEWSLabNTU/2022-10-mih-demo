mod kiss3d_gui;
mod opencv_gui;

use std::time::Duration;

use anyhow::Result;
use async_std::task::spawn_blocking;
use clap::Parser;
use futures::{future, future::FutureExt as _, stream::StreamExt as _};
use r2r::{
    sensor_msgs::msg::{Image, PointCloud},
    vision_msgs::msg::Detection2DArray,
    Context, Node, QosProfile,
};

#[derive(Parser)]
struct Opts {
    /// Input point cloud topic.
    #[clap(long, default_value = "pcd")]
    pub pcd_topic: String,

    /// Input detection 2D boxes topic.
    #[clap(long, default_value = "det")]
    pub det_topic: String,

    /// Input image topic.
    #[clap(long, default_value = "img")]
    pub img_topic: String,

    /// Namespace.
    #[clap(long, default_value = "/")]
    pub namespace: String,
}

#[async_std::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "demo_viz", &opts.namespace)?;

    let pcd_sub = node.subscribe::<PointCloud>(&opts.pcd_topic, QosProfile::default())?;
    let det_sub = node.subscribe::<Detection2DArray>(&opts.det_topic, QosProfile::default())?;
    let img_sub = node.subscribe::<Image>(&opts.img_topic, QosProfile::default())?;

    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    let (gui2d_future, gui2d_tx) = opencv_gui::start();
    let (gui3d_future, gui3d_tx) = kiss3d_gui::start();

    let pcd_forward = pcd_sub.map(Ok).forward(gui3d_tx.into_sink());
    let det_forward = det_sub
        .map(opencv_gui::Message::from)
        .map(Ok)
        .forward(gui2d_tx.clone().into_sink());
    let img_forward = img_sub
        .map(opencv_gui::Message::from)
        .map(Ok)
        .forward(gui2d_tx.into_sink());

    let join1 = future::try_join3(gui2d_future, gui3d_future, spin_future.map(Ok));
    let join2 = future::try_join3(
        pcd_forward.map(|result| result.map_err(|_| ())),
        det_forward.map(|result| result.map_err(|_| ())),
        img_forward.map(|result| result.map_err(|_| ())),
    );

    futures::select! {
        result = join1.fuse() => {
            result?;
        }
        _ = join2.fuse() => {}
    };

    Ok(())
}
