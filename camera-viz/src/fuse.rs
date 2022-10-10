use crate::{
    config::{Config, ExtrinsicsData, MrptCalibration},
    message as msg,
    rect_rtree::RectRTree,
};
use anyhow::{bail, ensure, Result};
use async_std::task::spawn_blocking;
use cv_convert::{FromCv, OpenCvPose};
use futures::prelude::*;
use itertools::{chain, izip};
use nalgebra as na;
use opencv::{
    calib3d,
    core::{no_array, Point2f, Point3f, Rect, Vector},
    prelude::*,
};
use ownref::ArcRefA as ARef;
use r2r::{
    geometry_msgs::msg::Pose2D,
    log_error,
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
        'msg_loop: while let Ok(in_msg) = input_rx.recv() {
            let result = state.update_msg(in_msg);
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

            for msg in out_msgs {
                let result = output_tx.send(msg);
                if result.is_err() {
                    break 'msg_loop;
                }
            }
        }
    });

    let output_stream = output_rx.into_stream().take_until(async move {
        futures::join!(forward_future, handle_future);
    });

    Ok(output_stream)
}

struct PointProjector {
    height: usize,
    width: usize,
    camera_params: CameraParams,
}

impl PointProjector {
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
        let width_range = 0.0..=(self.width as f32);
        let height_range = 0.0..=(self.height as f32);
        let inbound_points = point_pairs.filter(move |(_pcd_point, img_point)| {
            width_range.contains(&img_point.x) && height_range.contains(&img_point.y)
        });

        inbound_points
    }
}

struct State {
    cache: Cache,
    otobrite_projector: PointProjector,
    kneron_projector: PointProjector,
}

impl State {
    pub fn new(config: &Config) -> Result<Self> {
        let otobrite_projector = {
            let [h, w] = config.otobrite_image_hw;
            let camera_params = CameraParams::new(
                &config.otobrite_intrinsics_file,
                &config.otobrite_extrinsics_file,
            )?;

            PointProjector {
                height: h.get(),
                width: w.get(),
                camera_params,
            }
        };
        let kneron_projector = {
            let [h, w] = config.kneron_image_hw;
            let camera_params = CameraParams::new(
                &config.kneron_intrinsics_file,
                &config.kneron_extrinsics_file,
            )?;

            PointProjector {
                height: h.get(),
                width: w.get(),
                camera_params,
            }
        };

        Ok(Self {
            otobrite_projector,
            kneron_projector,
            cache: Cache::default(),
        })
    }

    pub fn update_msg(&mut self, in_msg: msg::InputMessage) -> Result<Vec<msg::FuseMessage>> {
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
                    rects: kneron_bboxes.as_ref().map(|bboxes| bboxes.rects.clone()),
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
                    rects: kneron_bboxes.as_ref().map(|bboxes| bboxes.rects.clone()),
                    assocs: kneron_assocs.clone(),
                }
                .into();

                chain!(kiss3d_msg, [kneron_msg]).collect()
            }
        };
        Ok(out_msgs)
    }

    pub fn update_kneron_det(&mut self, det: Detection2DArray) {
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

        self.cache.kneron_bboxes = Some(BBoxIndex {
            rects: rects.clone(),
            index,
        });
        self.update_kneron_assocs();
    }

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
        self.cache.otobrite_image = Some(mat);

        Ok(())
    }

    pub fn update_pcd(&mut self, pcd: PointCloud2) -> Result<()> {
        let points = pcd_to_points(&pcd)?;
        self.cache.points = Some(ARef::new(points));

        self.update_kneron_assocs();
        self.update_otobrite_assocs();
        Ok(())
    }

    fn update_otobrite_assocs(&mut self) {
        let points = match self.cache.points.as_ref() {
            Some(points) => points,
            None => return,
        };
        let assocs: Vec<_> = self
            .otobrite_projector
            .map(points)
            .map(|(pcd_point, img_point)| msg::Association {
                pcd_point,
                img_point,
                rect: None,
            })
            .collect();

        self.cache.otobrite_assocs = Some(ARef::new(assocs));
    }

    fn update_kneron_assocs(&mut self) {
        let points = match &self.cache.points {
            Some(points) => points,
            None => return,
        };
        let pairs = self.kneron_projector.map(points);

        let assocs: Vec<_> = match &self.cache.kneron_bboxes {
            Some(bboxes) => pairs
                .map(|(pcd_point, img_point)| {
                    let rect = bboxes.index.find(&img_point);
                    msg::Association {
                        pcd_point,
                        img_point,
                        rect,
                    }
                })
                .collect(),
            None => pairs
                .map(|(pcd_point, img_point)| msg::Association {
                    pcd_point,
                    img_point,
                    rect: None,
                })
                .collect(),
        };

        self.cache.kneron_assocs = Some(ARef::new(assocs));
    }
}

struct CameraParams {
    rvec: Mat,
    tvec: Mat,
    camera_matrix: Mat,
    distortion_coefficients: Mat,
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

#[derive(Default)]
struct Cache {
    points: Option<msg::ArcPointVec>,
    otobrite_image: Option<Mat>,
    kneron_bboxes: Option<BBoxIndex>,
    otobrite_assocs: Option<msg::ArcAssocVec>,
    kneron_assocs: Option<msg::ArcAssocVec>,
}

struct BBoxIndex {
    rects: msg::ArcRectVec,
    index: RectRTree,
}

pub fn pcd_to_points(pcd: &PointCloud2) -> Result<Vec<msg::Point>> {
    let [fx, fy, fz, fi] = match pcd.fields.get(0..4) {
        Some([f1, f2, f3, f4]) => [f1, f2, f3, f4],
        Some(_) => unreachable!(),
        None => {
            bail!("Ignore a point cloud message with less then 3 fields");
        }
    };

    if !(fx.name == "x" && fy.name == "y" && fz.name == "z" && fi.name == "intensity") {
        bail!("Ignore a point cloud message with incorrect field name");
    }

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

    if pcd.point_step != 16 {
        bail!("Ignore a point cloud message with incorrect point_step (expect 16)");
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

    Ok(points)
}

pub fn image_to_mat(image: &Image) -> Result<Mat> {
    use opencv::core::{Scalar, Vec3b, VecN, CV_8UC3};

    let Image {
        height,
        width,
        ref encoding,
        is_bigendian,
        step,
        ref data,
        ..
    } = *image;

    ensure!(encoding == "bgr8");
    ensure!(is_bigendian == 0);
    ensure!(step == width * 3);
    ensure!(data.len() == step as usize * height as usize);

    let mut mat =
        Mat::new_rows_cols_with_default(height as i32, width as i32, CV_8UC3, Scalar::all(0.0))?;

    data.chunks_exact(3).enumerate().for_each(|(pidx, bytes)| {
        let col = pidx % width as usize;
        let row = pidx / width as usize;
        let pixel: &mut Vec3b = mat.at_2d_mut(row as i32, col as i32).unwrap();
        let bytes: [u8; 3] = bytes.try_into().unwrap();
        *pixel = VecN(bytes);
    });

    Ok(mat)
}
