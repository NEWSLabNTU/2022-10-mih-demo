use std::mem;

use anyhow::{ensure, Result};
use nalgebra as na;
use noisy_float::prelude::*;
use serde::Deserialize;
use serde_loader::AbsPathBuf;
use serde_semver::SemverReq;

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

    /// Input topic for Autoware detected objects.
    pub aw_det_topic: String,

    /// The calibration file.
    pub calibration_file: AbsPathBuf,
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

#[derive(Debug, Clone, Deserialize)]
pub struct Matrix {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<R64>,
}

impl Matrix {
    pub fn to_na(&self) -> Result<na::DMatrix<f64>> {
        ensure!(
            self.rows * self.cols == self.data.len(),
            "data size does not match rows and cols"
        );
        let slice: &[f64] = unsafe { mem::transmute(self.data.as_slice()) };
        let mat = na::DMatrix::from_row_slice(self.rows, self.cols, slice);
        Ok(mat)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DistortionModel {
    PlumpBob,
}
