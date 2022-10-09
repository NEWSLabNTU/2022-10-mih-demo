use nalgebra as na;
use opencv::{
    core::{Point2f, Rect},
    prelude::*,
};
use ownref::ArcRefA as ARef;
use r2r::{
    sensor_msgs::msg::{Image, PointCloud2},
    vision_msgs::msg::Detection2DArray,
};

pub type ArcPointVec = ARef<'static, Vec<Point>>;
pub type ArcPoint = ARef<'static, Vec<Point>, Point>;
pub type ArcRectVec = ARef<'static, Vec<Rect>>;
pub type ArcRect = ARef<'static, Vec<Rect>, Rect>;
pub type ArcAssocVec = ARef<'static, Vec<Association>>;

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
    pub rects: ArcRectVec,
    pub assocs: Option<ArcAssocVec>,
}

#[derive(Debug)]
pub struct Kiss3dMessage {
    pub points: ArcPointVec,
    pub assocs: Option<ArcAssocVec>,
}

#[derive(Debug)]
pub struct Point {
    pub position: na::Point3<f32>,
    pub intensity: f32,
}

#[derive(Debug)]
pub struct Association {
    pub pcd_point: ArcPoint,
    pub img_point: Point2f,
    pub rect: Option<ArcRect>,
}
