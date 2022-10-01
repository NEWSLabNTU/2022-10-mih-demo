mod kiss3d_gui;
mod opencv_gui;

use anyhow::Result;
use futures::{future, future::FutureExt as _, stream::StreamExt as _};
use r2r::{
    sensor_msgs::msg::{Image, PointCloud},
    vision_msgs::msg::BoundingBox2DArray,
    Context, Node, QosProfile,
};

#[async_std::main]
async fn main() -> Result<()> {
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "demo_viz", "namespace")?;

    let pcd_sub = node.subscribe::<PointCloud>("PCD", QosProfile::default())?;
    let det_sub = node.subscribe::<BoundingBox2DArray>("DET", QosProfile::default())?;
    let img_sub = node.subscribe::<Image>("IMG", QosProfile::default())?;

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

    let join1 = future::try_join(gui2d_future, gui3d_future);
    let mut join2 = future::join3(
        pcd_forward.map(|_| ()),
        det_forward.map(|_| ()),
        img_forward.map(|_| ()),
    );

    futures::select! {
        result = join1.fuse() => {
            result?;
        }
        _ = join2 => {}
    };

    Ok(())
}
