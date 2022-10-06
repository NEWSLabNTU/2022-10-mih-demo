use nalgebra as na;
use opencv::{core::Rect, prelude::*};
use r2r::{
    sensor_msgs::msg::{Image, PointCloud2},
    vision_msgs::msg::Detection2DArray,
};
use std::sync::Arc;

#[derive(Debug)]
pub enum InputMessage {
    PointCloud2(PointCloud2),
    Image(Image),
    BBox(Detection2DArray),
}

impl From<Detection2DArray> for InputMessage {
    fn from(v: Detection2DArray) -> Self {
        Self::BBox(v)
    }
}

impl From<Image> for InputMessage {
    fn from(v: Image) -> Self {
        Self::Image(v)
    }
}

impl From<PointCloud2> for InputMessage {
    fn from(v: PointCloud2) -> Self {
        Self::PointCloud2(v)
    }
}

#[derive(Debug)]
pub struct FuseMessage {
    pub opencv_msg: Option<OpencvGuiMessage>,
    pub kiss3d_msg: Option<Kiss3dMessage>,
}

#[derive(Debug)]
pub struct OpencvGuiMessage {
    pub image: Mat,
    pub rects: Arc<Vec<Rect>>,
}

#[derive(Debug)]
pub struct Kiss3dMessage {
    pub points: Arc<Vec<Point>>,
}

#[derive(Debug)]
pub struct Point {
    pub position: na::Point3<f32>,
    pub intensity: f32,
}
