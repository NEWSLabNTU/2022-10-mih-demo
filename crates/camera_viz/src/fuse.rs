use std::slice;

use crate::{
    config::Config,
    message as msg,
    point_projection::{CameraParams, PointProjector},
};
use anyhow::{bail, ensure, Result};
use async_std::task::spawn_blocking;
use fast_yuv442_to_rgb24::uvy422_to_bgr24::uyvy422_to_bgr24_chunk16_many;
use futures::prelude::*;
use itertools::chain;
use nalgebra as na;
use opencv::{
    core::{Rect, ROTATE_180},
    prelude::*,
};
use ownref::ArcRefA as ARef;
use r2r::{
    geometry_msgs::msg::Pose2D,
    log_error,
    sensor_msgs::msg::{Image, PointCloud2, PointField},
    vision_msgs::msg::{BoundingBox2D, Detection2DArray},
};
use rayon::prelude::*;

/// Starts a image and point cloud fusing processor.
///
/// # Parameters
/// - `input_stream`: the stream to be transformed.
/// - `config`: Configuration data.
pub fn start(
    mut input_stream: impl Stream<Item = msg::InputMessage> + Unpin + Send,
    config: &Config,
) -> Result<impl Stream<Item = msg::FuseMessage> + Send> {
    // Initialize the state
    let mut state = State::new(config)?;

    // Create an input and an output channels.
    let (input_tx, input_rx) = flume::bounded(2);
    let (output_tx, output_rx) = flume::bounded(2);

    // Forward the stream to the input channel.
    let forward_future = async move {
        while let Some(msg) = input_stream.next().await {
            let result = input_tx.try_send(msg);
            match result {
                Ok(()) => {}
                Err(flume::TrySendError::Full(_)) => {}
                Err(flume::TrySendError::Disconnected(_)) => break,
            }
        }
    };

    // Spawn a non-async loop task that updates the state whenever a
    // message arrives.
    let handle_future = spawn_blocking(move || {
        'msg_loop: while let Ok(in_msg) = input_rx.recv() {
            // Update the state
            let result = state.map_msg(in_msg);

            // Print log if an error is returned
            let out_msgs = match result {
                Ok(msgs) => msgs,
                Err(err) => {
                    log_error!(
                        env!("CARGO_PKG_NAME"),
                        "Unable to process an input message: {:#}",
                        err
                    );
                    continue;
                }
            };

            // Forward the output messages to the output channel.
            for msg in out_msgs {
                let result = output_tx.send(msg);
                if result.is_err() {
                    break 'msg_loop;
                }
            }
        }
    });

    // Turn the receiver of the output channel to a stream and return it.
    let output_stream = output_rx.into_stream().take_until(async move {
        futures::join!(forward_future, handle_future);
    });

    Ok(output_stream)
}

/// The state maintained by the fusing algorithm.
struct State {
    cache: Cache,
    otobrite_projector: PointProjector,
    kneron_projector: PointProjector,
    otobrite_rotate_180: bool,
    kneron_scale_hw: [f64; 2],
}

impl State {
    /// Creata a new state.
    pub fn new(config: &Config) -> Result<Self> {
        let otobrite_projector = {
            let [h, w] = config.otobrite_image_hw;
            let camera_params =
                CameraParams::new(&config.otobrite_intrinsics_file, &config.otobrite_pose())?;

            PointProjector {
                height: h.get(),
                width: w.get(),
                camera_params,
            }
        };
        let kneron_projector = {
            let [h, w] = config.kneron_image_hw;
            let camera_params =
                CameraParams::new(&config.kneron_intrinsics_file, &config.kneron_pose())?;

            PointProjector {
                height: h.get(),
                width: w.get(),
                camera_params,
            }
        };

        let kneron_scale_hw = {
            let [image_h, image_w] = config.kneron_image_hw;
            let [det_h, det_w] = config.kneron_det_hw;
            [
                image_h.get() as f64 / det_h.get() as f64,
                image_w.get() as f64 / det_w.get() as f64,
            ]
        };

        Ok(Self {
            otobrite_projector,
            kneron_projector,
            otobrite_rotate_180: config.otobrite_image_rotate_180,
            cache: Cache::default(),
            kneron_scale_hw,
        })
    }

    /// Map an input message to a vec of output messages.
    pub fn map_msg(&mut self, in_msg: msg::InputMessage) -> Result<Vec<msg::FuseMessage>> {
        use msg::InputMessage as M;
        let out_msgs: Vec<msg::FuseMessage> = match in_msg {
            M::PointCloud2(pcd) => {
                self.update_pcd(pcd)?;

                let Cache {
                    points,
                    otobrite_image,
                    kneron_bboxes,
                    otobrite_assocs,
                    kneron_assocs,
                } = &self.cache;

                let kiss3d_msg: Option<msg::FuseMessage> = points.as_ref().map(|points| {
                    let points = points.clone();
                    let kneron_assocs = kneron_assocs.clone();
                    msg::Kiss3dMessage {
                        points,
                        kneron_assocs,
                    }
                    .into()
                });
                let kneron_msg: msg::FuseMessage = msg::KneronMessage {
                    objects: kneron_bboxes.as_ref().map(|bboxes| bboxes.objects.clone()),
                    assocs: kneron_assocs.clone(),
                }
                .into();
                let otobrite_msg: msg::FuseMessage = msg::OtobriteMessage {
                    image: otobrite_image.clone(),
                    assocs: otobrite_assocs.clone(),
                }
                .into();

                chain!(kiss3d_msg, [kneron_msg, otobrite_msg]).collect()
            }
            M::OtobriteImage(img) => {
                self.update_otobrite_image(img)?;

                let Cache {
                    otobrite_image,
                    otobrite_assocs,
                    ..
                } = &self.cache;

                let out_msg: msg::FuseMessage = msg::OtobriteMessage {
                    image: otobrite_image.clone(),
                    assocs: otobrite_assocs.clone(),
                }
                .into();
                vec![out_msg]
            }
            M::BBox(det) => {
                self.update_kneron_det(det);
                let Cache {
                    points,
                    kneron_bboxes,
                    kneron_assocs,
                    ..
                } = &self.cache;

                let kiss3d_msg: Option<msg::FuseMessage> = points.as_ref().map(|points| {
                    let points = points.clone();
                    let kneron_assocs = kneron_assocs.clone();
                    msg::Kiss3dMessage {
                        points,
                        kneron_assocs,
                    }
                    .into()
                });
                let kneron_msg: msg::FuseMessage = msg::KneronMessage {
                    objects: kneron_bboxes.as_ref().map(|bboxes| bboxes.objects.clone()),
                    assocs: kneron_assocs.clone(),
                }
                .into();

                chain!(kiss3d_msg, [kneron_msg]).collect()
            }
        };
        Ok(out_msgs)
    }

    /// Processes a Kneron detection message and updates its state.
    pub fn update_kneron_det(&mut self, det: Detection2DArray) {
        let [scale_h, scale_w] = self.kneron_scale_hw;

        let objects: Vec<_> = det
            .detections
            .par_iter()
            .map(|det| {
                let class_id = det
                    .results
                    .get(0)
                    .map(|res| res.hypothesis.class_id.clone());
                let BoundingBox2D {
                    size_x,
                    size_y,
                    center: Pose2D { x: cx, y: cy, .. },
                } = det.bbox;

                // left-top x and y
                let ltx = (cx - size_x / 2.0) * scale_w;
                let lty = (cy - size_y / 2.0) * scale_h;

                let width = size_x * scale_w;
                let height = size_y * scale_h;

                msg::Object {
                    class_id,
                    rect: Rect {
                        x: ltx as i32,
                        y: lty as i32,
                        width: width as i32,
                        height: height as i32,
                    },
                }
            })
            .collect();
        let objects = ARef::new(objects);
        // let index: RectRTree = objects.clone().flatten().collect();

        self.cache.kneron_bboxes = Some(BBoxIndex {
            objects: objects.clone(),
            // index,
        });
        self.update_kneron_assocs();
    }

    /// Processes an image from the Otobrite camera.
    pub fn update_otobrite_image(&mut self, image: Image) -> Result<()> {
        // Check image size
        {
            let expect_h = self.otobrite_projector.height;
            let expect_w = self.otobrite_projector.width;
            let image_h = image.height;
            let image_w = image.width;

            let ok = expect_h as u32 == image_h && expect_w as u32 == image_w;
            ensure!(
                ok,
                "Expect {}x{} sized image, but received an image with size {}x{}",
                expect_w,
                expect_h,
                image_w,
                image_h
            );
        }

        let mat = image_to_mat(&image)?;
        let mat = if self.otobrite_rotate_180 {
            let mut out = Mat::default();
            opencv::core::rotate(&mat, &mut out, ROTATE_180)?;
            out
        } else {
            mat
        };

        self.cache.otobrite_image = Some(mat);

        Ok(())
    }

    /// Processes a point cloud message from LiDAR.
    pub fn update_pcd(&mut self, pcd: PointCloud2) -> Result<()> {
        let points = pcd_to_points(&pcd)?;
        self.cache.points = Some(ARef::new(points));

        self.update_kneron_assocs();
        self.update_otobrite_assocs();
        Ok(())
    }

    /// Compute LiDAR points to Otobrite image points associations.
    fn update_otobrite_assocs(&mut self) {
        let points = match self.cache.points.as_ref() {
            Some(points) => points,
            None => return,
        };
        let assocs: Vec<_> = self
            .otobrite_projector
            .project(points)
            .into_par_iter()
            .map(|(pcd_point, img_point)| msg::Association {
                pcd_point,
                img_point,
                object: None,
            })
            .collect();

        self.cache.otobrite_assocs = Some(ARef::new(assocs));
    }

    /// Compute LiDAR points to Kneron image points associations.
    fn update_kneron_assocs(&mut self) {
        let points = match &self.cache.points {
            Some(points) => points,
            None => return,
        };

        // Compute projected 2D points.
        let pairs = self.kneron_projector.project(points);

        // Associate points with bboxes if bboxes are available.
        let assocs: Vec<msg::Association> = match &self.cache.kneron_bboxes {
            Some(bboxes) => pairs
                .into_par_iter()
                .map(|(pcd_point, img_point)| {
                    let object = bboxes.objects.clone().flatten().find(|object| {
                        let Rect {
                            x: lx,
                            y: ty,
                            width: w,
                            height: h,
                        } = object.rect;
                        let rx = lx + w;
                        let by = ty + h;

                        let x_range = (lx as f32)..=(rx as f32);
                        let y_range = (ty as f32)..=(by as f32);

                        x_range.contains(&img_point.x) && y_range.contains(&img_point.y)
                    });

                    msg::Association {
                        pcd_point,
                        img_point,
                        object,
                    }
                })
                .collect(),
            None => pairs
                .into_par_iter()
                .map(|(pcd_point, img_point)| msg::Association {
                    pcd_point,
                    img_point,
                    object: None,
                })
                .collect(),
        };

        self.cache.kneron_assocs = Some(ARef::new(assocs));
    }
}

/// The cache stores computed point cloud, image and detection  data.
#[derive(Default)]
struct Cache {
    points: Option<msg::ArcPointVec>,
    otobrite_image: Option<Mat>,
    kneron_bboxes: Option<BBoxIndex>,
    otobrite_assocs: Option<msg::ArcAssocVec>,
    kneron_assocs: Option<msg::ArcAssocVec>,
}

/// Contains a vec of bboxes and a spatial R-Tree of bboxes.
struct BBoxIndex {
    objects: msg::ArcObjVec,
    // index: RectRTree,
}

/// Converts a ROS point cloud to a vec of points.
pub fn pcd_to_points(pcd: &PointCloud2) -> Result<Vec<msg::Point>> {
    // Assert the point cloud has at least 4 fields. Otherwise return error.
    let [fx, fy, fz, fi] = match pcd.fields.get(0..4) {
        Some([f1, f2, f3, f4]) => [f1, f2, f3, f4],
        Some(_) => unreachable!(),
        None => {
            bail!("Ignore a point cloud message with less then 3 fields");
        }
    };

    // Assert the fields are named x, y, z and intensity. Otherwise return error.
    if !(fx.name == "x" && fy.name == "y" && fz.name == "z" && fi.name == "intensity") {
        bail!("Ignore a point cloud message with incorrect field name");
    }

    // Assert each field is of f32 type and contains a single value. Otherwise, return error.
    let check_field = |field: &PointField| {
        let PointField {
            datatype, count, ..
        } = *field;

        // reject non-f64 or non-single-value fields
        if !(datatype == 7 && count == 1) {
            bail!("Ignore a point cloud message with non-f64 or non-single-value values");
        }

        anyhow::Ok(())
    };

    check_field(fx)?;
    check_field(fy)?;
    check_field(fz)?;
    check_field(fi)?;

    // Assert a point is 16 bytes (4 x f32 values). Otherwise, return error.
    if pcd.point_step < 16 {
        bail!("Ignore a point cloud message with incorrect point_step (expect 16)");
    }

    // Transform the data byte to a vec of points.
    let points: Vec<msg::Point> = pcd
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

    Ok(points)
}

/// Converts a ROS image to an OpenCV Mat.
pub fn image_to_mat(image: &Image) -> Result<Mat> {
    use opencv::core::{Scalar, Vec3b, VecN, CV_8UC3};

    let Image {
        height,
        width,
        ref encoding,
        is_bigendian,
        step: row_step,
        ref data,
        ..
    } = *image;

    let is_bigendian = is_bigendian != 0;
    ensure!(!is_bigendian);

    let mat = match encoding.as_str() {
        "BGR8" => {
            let pixel_step = 3;
            ensure!(row_step == width * pixel_step);
            ensure!(data.len() == (row_step * height) as usize);

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

            mat
        }
        "RGB8" => {
            let pixel_step = 3;
            ensure!(row_step == width * pixel_step);
            ensure!(data.len() == (row_step * height) as usize);

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
                let [r, g, b]: [u8; 3] = bytes.try_into().unwrap();
                *pixel = VecN([b, g, r]);
            });

            mat
        }
        "UYVY" => unsafe {
            let pixel_step = 2;
            ensure!(row_step == width * pixel_step);
            ensure!(data.len() == (row_step * height) as usize);

            let mut mat = Mat::new_rows_cols(height as i32, width as i32, CV_8UC3)?;
            let uyvy422_buf = data;
            let bgr24_buf =
                slice::from_raw_parts_mut(mat.data_mut(), (height * width * 3) as usize);
            uyvy422_to_bgr24_chunk16_many(uyvy422_buf, bgr24_buf);

            mat
        },
        _ => bail!("unsupported image format {}", encoding),
    };

    Ok(mat)
}
