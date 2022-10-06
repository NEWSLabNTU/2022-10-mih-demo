use crate::rect_rtree::RectRTree;
use crate::{config::Config, message as msg};
use anyhow::{ensure, Result};
use async_std::task::spawn_blocking;
use cv_convert::FromCv;
use cv_convert::OpenCvPose;
use futures::prelude::*;
use itertools::izip;
use nalgebra as na;
use opencv::calib3d;
use opencv::core::Point2f;
use opencv::core::Vector;
use opencv::{
    core::{Point3f, Rect},
    prelude::*,
};
use ownref::ArcRefA as ARef;
use r2r::{
    geometry_msgs::msg::Pose2D,
    log_error, log_warn,
    sensor_msgs::msg::{Image, PointCloud2, PointField},
    vision_msgs::msg::{BoundingBox2D, Detection2DArray},
};

pub fn start(
    input_stream: impl Stream<Item = msg::InputMessage> + Unpin + Send,
    config: &Config,
) -> Result<impl Stream<Item = msg::FuseMessage> + Send> {
    let mut state = State::new(config)?;
    let (input_tx, input_rx) = flume::bounded(2);
    let (output_tx, output_rx) = flume::bounded(2);

    let forward_future = input_stream
        .map(Ok)
        .forward(input_tx.into_sink())
        .map(|_result| ());

    let handle_future = spawn_blocking(move || {
        while let Ok(msg) = input_rx.recv() {
            let result = state.update_msg(msg);
            if let Err(err) = result {
                log_error!(
                    env!("CARGO_PKG_NAME"),
                    "Unable to process an input message: {:#}",
                    err
                );
                continue;
            }

            let output = state.step();
            let result = output_tx.send(output);
            if result.is_err() {
                break;
            }
        }
    });

    let output_stream = output_rx.into_stream().take_until(async move {
        futures::join!(forward_future, handle_future);
    });

    Ok(output_stream)
}

struct State {
    image: Option<Mat>,
    points: Option<msg::ArcPointVec>,
    rect_state: Option<RectState>,
    rvec: Mat,
    tvec: Mat,
    camera_matrix: Mat,
    distortion_coefficients: Mat,
}

impl State {
    pub fn new(config: &Config) -> Result<Self> {
        let OpenCvPose { rvec, tvec } = config.extrinsics_file.to_opencv()?;
        let camera_matrix = config.intrinsics_file.camera_matrix.to_opencv();
        let distortion_coefficients = config.intrinsics_file.distortion_coefficients.to_opencv();

        Ok(Self {
            image: None,
            points: None,
            rect_state: None,
            rvec,
            tvec,
            camera_matrix,
            distortion_coefficients,
        })
    }

    pub fn update_msg(&mut self, msg: msg::InputMessage) -> Result<()> {
        use msg::InputMessage as M;
        match msg {
            M::PointCloud2(pcd) => self.update_pcd(pcd),
            M::Image(img) => self.update_image(img)?,
            M::BBox(det) => self.update_det(det),
        }
        Ok(())
    }

    pub fn update_det(&mut self, det: Detection2DArray) {
        let rects: Vec<_> = det
            .detections
            .iter()
            .map(|det| {
                let BoundingBox2D {
                    size_x,
                    size_y,
                    center: Pose2D { x: cx, y: cy, .. },
                } = det.bbox;

                // left-top x and y
                let ltx = cx - size_x / 2.0;
                let lty = cy - size_y / 2.0;

                Rect {
                    x: ltx as i32,
                    y: lty as i32,
                    width: size_x as i32,
                    height: size_y as i32,
                }
            })
            .collect();
        let rects = ARef::new(rects);
        let index: RectRTree = rects.clone().flatten().collect();

        self.rect_state = Some(RectState { rects, index });
    }

    pub fn update_image(&mut self, image: Image) -> Result<()> {
        use opencv::core::{Scalar, Vec3b, VecN, CV_8UC3};

        let Image {
            height,
            width,
            encoding,
            is_bigendian,
            step,
            data,
            ..
        } = image;

        ensure!(encoding == "bgr8");
        ensure!(is_bigendian == 0);
        ensure!(step == width * 3);
        ensure!(data.len() == step as usize * height as usize);

        let mut mat = Mat::new_rows_cols_with_default(
            height as i32,
            width as i32,
            CV_8UC3,
            Scalar::all(0.0),
        )?;

        data.chunks_exact(3).enumerate().for_each(|(pidx, bytes)| {
            let col = pidx % width as usize;
            let row = pidx / width as usize;
            let pixel: &mut Vec3b = mat.at_2d_mut(row as i32, col as i32).unwrap();
            let bytes: [u8; 3] = bytes.try_into().unwrap();
            *pixel = VecN(bytes);
        });

        self.image = Some(mat);
        Ok(())
    }

    pub fn update_pcd(&mut self, pcd: PointCloud2) {
        let [fx, fy, fz, fi] = match pcd.fields.get(0..4) {
            Some([f1, f2, f3, f4]) => [f1, f2, f3, f4],
            Some(_) => unreachable!(),
            None => {
                log_warn!(
                    env!("CARGO_PKG_NAME"),
                    "Ignore a point cloud message with less then 3 fields"
                );
                return;
            }
        };

        if !(fx.name == "x" && fy.name == "y" && fz.name == "z" && fi.name == "intensity") {
            log_warn!(
                env!("CARGO_PKG_NAME"),
                "Ignore a point cloud message with incorrect field name"
            );
            return;
        }

        let check_field = |field: &PointField| {
            let PointField {
                datatype, count, ..
            } = *field;

            // reject non-f64 or non-single-value fields
            if !(datatype == 7 && count == 1) {
                log_warn!(
                    env!("CARGO_PKG_NAME"),
                    "Ignore a point cloud message with non-f64 or non-single-value values"
                );
                return false;
            }

            true
        };
        if !(check_field(fx) && check_field(fy) && check_field(fz) && check_field(fi)) {
            return;
        }

        if pcd.point_step != 16 {
            log_warn!(
                env!("CARGO_PKG_NAME"),
                "Ignore a point cloud message with incorrect point_step (expect 16)"
            );
            return;
        }

        let points: Vec<_> = pcd
            .data
            .chunks(pcd.point_step as usize)
            .map(|point_bytes| {
                let xbytes = &point_bytes[0..4];
                let ybytes = &point_bytes[4..8];
                let zbytes = &point_bytes[8..12];
                let ibytes = &point_bytes[12..16];

                let x = f32::from_le_bytes(xbytes.try_into().unwrap());
                let y = f32::from_le_bytes(ybytes.try_into().unwrap());
                let z = f32::from_le_bytes(zbytes.try_into().unwrap());
                let position = na::Point3::new(x, y, z);
                let intensity = f32::from_le_bytes(ibytes.try_into().unwrap());

                msg::Point {
                    position,
                    intensity,
                }
            })
            .collect();

        self.points = Some(ARef::new(points));
    }

    pub fn step(&self) -> msg::FuseMessage {
        use opencv::core::no_array;

        let Self {
            image,
            points,
            rect_state,
            rvec,
            tvec,
            camera_matrix,
            distortion_coefficients,
            ..
        } = self;

        // Compute 3D point, 2D point and bbox associations
        let assocs: Option<msg::ArcAssocVec> =
            if let (Some(points), Some(image), Some(rect_state)) = (points, image, rect_state) {
                // Project points onto the image
                let object_points: Vector<Point3f> = points
                    .iter()
                    .map(|point| &point.position)
                    .map(Point3f::from_cv)
                    .collect();
                let mut image_points: Vector<Point2f> = Vector::new();

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
                let width_range = 0.0..=(image.cols() as f32);
                let height_range = 0.0..=(image.rows() as f32);
                let inbound_points = point_pairs.filter(|(_pcd_point, img_point)| {
                    width_range.contains(&img_point.x) && height_range.contains(&img_point.y)
                });

                // Pair up each 2D point with at most one bbox
                let assocs = inbound_points.map(|(pcd_point, img_point)| {
                    let rect = rect_state.index.find(&img_point);
                    msg::Association {
                        pcd_point,
                        img_point,
                        rect,
                    }
                });

                let assoc_vec: Vec<_> = assocs.collect();
                Some(ARef::new(assoc_vec))
            } else {
                None
            };

        // Construct the output message
        {
            let opencv_msg = if let (Some(image), Some(rect_state)) = (image, rect_state) {
                Some(msg::OpencvGuiMessage {
                    image: image.clone(),
                    rects: rect_state.rects.clone(),
                    assocs: assocs.clone(),
                })
            } else {
                None
            };
            let kiss3d_msg = points.as_ref().map(|points| msg::Kiss3dMessage {
                points: points.clone(),
                assocs,
            });

            msg::FuseMessage {
                opencv_msg,
                kiss3d_msg,
            }
        }
    }
}

struct RectState {
    rects: msg::ArcRectVec,
    index: RectRTree,
}
