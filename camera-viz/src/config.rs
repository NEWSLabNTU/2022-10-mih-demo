use crate::yaml_loader::YamlPath;
use anyhow::Result;
use cv_convert::{OpenCvPose, TryIntoCv};
use nalgebra as na;
use noisy_float::prelude::*;
use opencv::prelude::*;
use serde::{de::Error as _, Deserialize, Deserializer};
use serde_loader::Json5Path;
use serde_semver::SemverReq;
use slice_of_array::prelude::*;
use std::mem;

#[derive(Debug, Clone, SemverReq)]
#[version("0.1.0")]
pub struct Version;

#[derive(Debug, Clone, Deserialize)]
pub struct Config {
    /// Config format version.
    pub version: Version,

    /// ROS Namespace.
    pub namespace: String,

    /// Input topic for point cloud.
    pub pcd_topic: String,

    /// Input topic for image.
    pub img_topic: String,

    /// Input topic for 2D detected objects.
    pub det_topic: String,

    /// The intrinsic parameters file.
    pub intrinsics_file: YamlPath<MrptCalibration>,

    /// The extrinsic parameters file.
    pub extrinsics_file: Json5Path<ExtrinsicsData>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MrptCalibration {
    pub camera_name: String,
    pub focal_length_meters: R64,
    pub image_height: usize,
    pub image_width: usize,
    pub distortion_model: DistortionModel,
    pub distortion_coefficients: Matrix,
    pub camera_matrix: Matrix,
    pub projection_model: Matrix,
    pub rectification_matrix: Matrix,
}

#[derive(Debug, Clone)]
pub struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<R64>,
}

impl Matrix {
    pub fn to_opencv(&self) -> Mat {
        let mat = Mat::from_slice(self.data_f64()).unwrap();
        let mat = mat.reshape(1, self.rows as i32).unwrap();
        mat
    }

    pub fn to_na(&self) -> na::DMatrix<f64> {
        na::DMatrix::from_row_slice(self.rows, self.cols, self.data_f64())
    }

    /// Get the matrix's rows.
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Get the matrix's cols.
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Get a reference to the matrix's data.
    pub fn data(&self) -> &[R64] {
        self.data.as_ref()
    }

    pub fn data_f64(&self) -> &[f64] {
        unsafe { mem::transmute(self.data()) }
    }
}

impl<'de> Deserialize<'de> for Matrix {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let UncheckedMatrix { rows, cols, data } = UncheckedMatrix::deserialize(deserializer)?;
        if rows * cols != data.len() {
            return Err(D::Error::custom(format!(
                "data size ({}) does not match rows ({}) and cols ({})",
                data.len(),
                rows,
                cols
            )));
        }
        Ok(Self { rows, cols, data })
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct UncheckedMatrix {
    rows: usize,
    cols: usize,
    data: Vec<R64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DistortionModel {
    PlumpBob,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ExtrinsicsData {
    Quaternion(ExtrinsicsTransform),
    Matrix(ExtrinsicsMatrix),
}

impl ExtrinsicsData {
    pub fn to_na(&self) -> na::Isometry3<f64> {
        match self {
            Self::Quaternion(me) => me.to_na(),
            Self::Matrix(me) => me.to_na(),
        }
    }

    pub fn to_opencv(&self) -> Result<OpenCvPose<Mat>> {
        let pose = self.to_na().try_into_cv()?;
        Ok(pose)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ExtrinsicsTransform {
    pub rot_wijk: [R64; 4],
    pub trans_xyz: [R64; 3],
}

impl ExtrinsicsTransform {
    pub fn to_na(&self) -> na::Isometry3<f64> {
        let Self {
            rot_wijk,
            trans_xyz,
        } = *self;
        let [w, i, j, k]: [f64; 4] = unsafe { mem::transmute(rot_wijk) };
        let [x, y, z]: [f64; 3] = unsafe { mem::transmute(trans_xyz) };

        let rotation = na::UnitQuaternion::from_quaternion(na::Quaternion::new(w, i, j, k));
        let translation = na::Translation3::new(x, y, z);

        na::Isometry3 {
            rotation,
            translation,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ExtrinsicsMatrix {
    pub rot: [[R64; 3]; 3],
    pub trans: [R64; 3],
}

impl ExtrinsicsMatrix {
    pub fn to_na(&self) -> na::Isometry3<f64> {
        let rotation = {
            let slice: &[R64] = self.rot.flat();
            let slice: &[f64] = unsafe { mem::transmute(slice) };
            let mat = na::Matrix3::from_row_slice(slice);
            na::UnitQuaternion::from_matrix(&mat)
        };
        let translation = {
            let [x, y, z]: [f64; 3] = unsafe { mem::transmute(self.trans) };
            na::Translation3::new(x, y, z)
        };
        na::Isometry3 {
            rotation,
            translation,
        }
    }
}
