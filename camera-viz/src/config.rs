use crate::yaml_loader::YamlPath;
use anyhow::Result;
use nalgebra as na;
use noisy_float::prelude::*;
use opencv::prelude::*;
use serde::{de::Error as _, Deserialize, Deserializer};
use serde_semver::SemverReq;
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

    /// The calibration file.
    pub calibration_file: YamlPath<MrptCalibration>,
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
