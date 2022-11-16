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

// Reference counted type aliases used for convenience.

pub type ArcPointVec = ARef<'static, Vec<Point>>;
pub type ArcPoint = ARef<'static, Vec<Point>, Point>;
pub type ArcObjVec = ARef<'static, Vec<Object>>;
pub type ArcObj = ARef<'static, Vec<Object>, Object>;
pub type ArcAssocVec = ARef<'static, Vec<Association>>;

/// An input message that can be a point cloud from LiDAR, an image
/// from the Otobrite camera or an image from Kneron camera.
#[derive(Debug)]
pub enum InputMessage {
    PointCloud2(PointCloud2),
    OtobriteImage(Image),
    BBox(Detection2DArray),
}

/// A message produced by the pcd/image fusing algorithm.
#[derive(Debug)]
pub enum FuseMessage {
    Otobrite(OtobriteMessage),
    Kneron(KneronMessage),
    Kiss3d(Kiss3dMessage),
}

impl From<Kiss3dMessage> for FuseMessage {
    fn from(v: Kiss3dMessage) -> Self {
        Self::Kiss3d(v)
    }
}

impl From<KneronMessage> for FuseMessage {
    fn from(v: KneronMessage) -> Self {
        Self::Kneron(v)
    }
}

impl From<OtobriteMessage> for FuseMessage {
    fn from(v: OtobriteMessage) -> Self {
        Self::Otobrite(v)
    }
}

/// A message containing an Otobrite image and projected LiDAR points.
#[derive(Debug)]
pub struct OtobriteMessage {
    pub image: Option<Mat>,
    pub assocs: Option<ArcAssocVec>,
}

/// A message containing bboxes from the Kneron camera and projected
/// LiDAR points.
#[derive(Debug)]
pub struct KneronMessage {
    pub objects: Option<ArcObjVec>,
    pub assocs: Option<ArcAssocVec>,
}

/// A message that is sent to Kiss3d GUI.
#[derive(Debug)]
pub struct Kiss3dMessage {
    pub points: ArcPointVec,
    pub kneron_assocs: Option<ArcAssocVec>,
}

/// A message that is sent to OpenCV GUI.
#[derive(Debug)]
pub enum OpencvMessage {
    Otobrite(OtobriteMessage),
    Kneron(KneronMessage),
}

impl From<KneronMessage> for OpencvMessage {
    fn from(v: KneronMessage) -> Self {
        Self::Kneron(v)
    }
}

impl From<OtobriteMessage> for OpencvMessage {
    fn from(v: OtobriteMessage) -> Self {
        Self::Otobrite(v)
    }
}

/// A point with a 3D position and an intensity.
#[derive(Debug)]
pub struct Point {
    pub position: na::Point3<f32>,
    pub intensity: f32,
}

/// Contains a 3D point with an associated 2D point and an associated bbox.
#[derive(Debug)]
pub struct Association {
    pub pcd_point: ArcPoint,
    pub img_point: Point2f,
    pub object: Option<ArcObj>,
}

/// An object containing a bounding box and a class ID.
#[derive(Debug)]
pub struct Object {
    pub rect: Rect,
    pub class_id: Option<String>,
}
