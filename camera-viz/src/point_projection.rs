use crate::{
    config::{ExtrinsicsData, MrptCalibration},
    message as msg,
};
use anyhow::Result;
use cv_convert::{FromCv, OpenCvPose};
use itertools::izip;
use opencv::{
    calib3d,
    core::{no_array, Point2f, Point3f, Vector},
    prelude::*,
};

/// An utility struct that projects 3D points to 2D space.
pub struct PointProjector {
    pub height: usize,
    pub width: usize,
    pub camera_params: CameraParams,
}

impl PointProjector {
    /// Projects a vec of 3D points to 2D space, and returns an
    /// iterator of 2D points.
    pub fn map(
        &self,
        points: &msg::ArcPointVec,
    ) -> impl Iterator<Item = (msg::ArcPoint, Point2f)> + Send {
        let CameraParams {
            rvec,
            tvec,
            camera_matrix,
            distortion_coefficients,
        } = &self.camera_params;

        // Convert input 3D points to OpenCV Point3f type.
        let object_points: Vector<Point3f> = points
            .iter()
            .map(|point| &point.position)
            .map(Point3f::from_cv)
            .collect();

        // Create a vector of 2D points that will be populated.
        let mut image_points: Vector<Point2f> = Vector::new();

        // Project points onto the image
        calib3d::project_points(
            &object_points,
            rvec,
            tvec,
            camera_matrix,
            distortion_coefficients,
            &mut image_points,
            &mut no_array(), // jacobian
            0.0,             // aspect_ratio
        )
        .unwrap();

        // Pair up 3D and 2D points
        let point_pairs = izip!(points.clone().flatten(), image_points);

        // Filter out out-of-bound projected points
        let width_range = 0.0..=(self.width as f32);
        let height_range = 0.0..=(self.height as f32);

        point_pairs.filter(move |(_pcd_point, img_point)| {
            width_range.contains(&img_point.x) && height_range.contains(&img_point.y)
        })
    }
}

/// Stores the intrinsic and extrinsic parameters of a camera.
pub struct CameraParams {
    pub rvec: Mat,
    pub tvec: Mat,
    pub camera_matrix: Mat,
    pub distortion_coefficients: Mat,
}

impl CameraParams {
    pub fn new(intrinsics: &MrptCalibration, extrinsics: &ExtrinsicsData) -> Result<Self> {
        let OpenCvPose { rvec, tvec } = extrinsics.to_opencv()?;
        let camera_matrix = intrinsics.camera_matrix.to_opencv();
        let distortion_coefficients = intrinsics.distortion_coefficients.to_opencv();
        Ok(Self {
            rvec,
            tvec,
            camera_matrix,
            distortion_coefficients,
        })
    }
}
