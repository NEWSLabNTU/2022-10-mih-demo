use anyhow::Result;
use async_std::task::{spawn_blocking, JoinHandle};
use itertools::Itertools as _;
use kiss3d::{
    camera::{ArcBall, Camera},
    event::{Action, Key, Modifiers, WindowEvent},
    light::Light,
    nalgebra as na,
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    window::Window,
};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use r2r::{
    autoware_auto_perception_msgs::msg::{DetectedObject, DetectedObjects, ObjectClassification},
    geometry_msgs::msg::{Point, Quaternion, Vector3},
    log_warn,
    sensor_msgs::msg::{PointCloud2, PointField},
};

pub fn start() -> (JoinHandle<Result<()>>, flume::Sender<Message>) {
    let (tx, rx) = flume::bounded(2);

    let handle = spawn_blocking(move || {
        let window = {
            let mut window = Window::new("demo");
            window.set_light(Light::StickToCamera);
            window
        };
        let mut camera = ArcBall::new(
            na::Point3::new(0.0, -80.0, 32.0),
            na::Point3::new(0.0, 0.0, 0.0),
        );
        camera.set_up_axis(na::Vector3::new(0.0, 0.0, 1.0));
        let state = State {
            points: vec![],
            objects: vec![],
            rx,
            camera,
            point_color_mode: PointColorMode::default(),
        };
        window.render_loop(state);
        anyhow::Ok(())
    });

    (handle, tx)
}

struct State {
    point_color_mode: PointColorMode,
    points: Vec<ColoredPoint>,
    objects: Vec<Object3D>,
    rx: flume::Receiver<Message>,
    camera: ArcBall,
}

impl State {
    fn process_events(&mut self, window: &mut Window) {
        window.events().iter().for_each(|evt| {
            use Action as A;
            use Key as K;
            use Modifiers as M;
            use WindowEvent as E;

            match evt.value {
                E::Key(key, action, mods) => {
                    let control = !(mods & M::Control).is_empty();
                    let shift = !(mods & M::Shift).is_empty();
                    let super_ = !(mods & M::Super).is_empty();

                    match (key, action, control, shift, super_) {
                        (K::C, A::Press, false, false, false) => {}
                        _ => {}
                    }
                }
                _ => {}
            }
        });
    }

    fn process_key_event() {}

    fn update_msg(&mut self, msg: Message) {
        match msg {
            Message::PointCloud2(pcd) => self.update_point_cloud(pcd),
            Message::DetectedObjects(objs) => self.update_aw_objs(objs),
        }
    }

    fn update_point_cloud(&mut self, pcd: PointCloud2) {
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

        self.points = pcd
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
                let intensity = f32::from_le_bytes(ibytes.try_into().unwrap());

                let position = na::Point3::new(x, y, z);

                let nint = intensity / 100.0; // normalized intensity
                                              // let color = na::Point3::new(nint, nint, nint);
                let color = na::Point3::new(0.3, 0.3, 0.3);
                ColoredPoint { position, color }
            })
            .collect();
    }

    fn update_aw_objs(&mut self, objs: DetectedObjects) {
        self.objects = objs.objects.iter().map(Object3D::from).collect();
    }

    fn render(&self, window: &mut Window) {
        // Draw axis
        self.draw_axis(window);

        // Draw points
        self.points.iter().for_each(|point| {
            let ColoredPoint { position, color } = point;
            window.draw_point(position, color);
        });

        // Draw 3D bbox segments
        self.objects.iter().for_each(|obj| {
            let ColoredSegmentSet { segments, color } = &obj.bbox_segments;
            segments.iter().for_each(|[pa, pb]| {
                window.draw_line(pa, pb, color);
            });
        });
    }

    fn draw_axis(&self, window: &mut Window) {
        let origin = na::Point3::new(0.0, 0.0, 0.0);
        window.draw_line(
            &origin,
            &na::Point3::new(1.0, 0.0, 0.0),
            &na::Point3::new(1.0, 0.0, 0.0),
        );
        window.draw_line(
            &origin,
            &na::Point3::new(0.0, 1.0, 0.0),
            &na::Point3::new(0.0, 1.0, 0.0),
        );
        window.draw_line(
            &origin,
            &na::Point3::new(0.0, 0.0, 1.0),
            &na::Point3::new(0.0, 0.0, 1.0),
        );
    }
}

impl kiss3d::window::State for State {
    fn step(&mut self, window: &mut Window) {
        // Try to receive a message
        match self.rx.try_recv() {
            Ok(msg) => {
                // update GUI state
                self.update_msg(msg);

                // if let Err(err) = result {
                //     window.close();
                //     log_error!(env!("CARGO_PKG_NAME"), "kiss3d gui error: {}", err);
                //     return;
                // }
            }
            Err(flume::TryRecvError::Empty) => {}
            Err(flume::TryRecvError::Disconnected) => {
                window.close();
                return;
            }
        }

        self.render(window);
    }

    #[allow(clippy::type_complexity)]
    fn cameras_and_effect(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (Some(&mut self.camera), None, None)
    }
}

struct ColoredPoint {
    pub position: na::Point3<f32>,
    pub color: na::Point3<f32>,
}

#[derive(Clone)]
struct ColoredSegmentSet {
    pub segments: Vec<[na::Point3<f32>; 2]>,
    pub color: na::Point3<f32>,
}

pub enum Message {
    PointCloud2(PointCloud2),
    DetectedObjects(DetectedObjects),
}

impl From<DetectedObjects> for Message {
    fn from(v: DetectedObjects) -> Self {
        Self::DetectedObjects(v)
    }
}

impl From<PointCloud2> for Message {
    fn from(v: PointCloud2) -> Self {
        Self::PointCloud2(v)
    }
}

struct Object3D {
    bbox_segments: ColoredSegmentSet,
    transform: na::Isometry3<f32>,
    size_xyz: [f32; 3],
    classification: Vec<ObjectClassification>,
}

impl Object3D {
    pub fn contains_point(&self, point: &na::Point3<f32>) -> bool {
        let point = self.transform.inverse() * point;
        let [sx, sy, sz] = self.size_xyz;

        let check_range = |sz: f32, value: f32| {
            let hsz = sz / 2.0;
            ((-hsz)..=hsz).contains(&value)
        };

        check_range(sx, point.x) && check_range(sy, point.y) && check_range(sz, point.z)
    }
}

impl From<DetectedObject> for Object3D {
    fn from(iobj: DetectedObject) -> Self {
        Self::from(&iobj)
    }
}

impl From<&DetectedObject> for Object3D {
    fn from(iobj: &DetectedObject) -> Self {
        // construct transform
        let transform = {
            // get position
            let translation = {
                let Point { x, y, z } = iobj.kinematics.pose_with_covariance.pose.position;
                na::Translation3::new(x as f32, y as f32, z as f32)
            };

            // get position
            let rotation = {
                let Quaternion { x, y, z, w } =
                    iobj.kinematics.pose_with_covariance.pose.orientation;
                na::UnitQuaternion::from_quaternion(na::Quaternion::new(
                    w as f32, x as f32, y as f32, z as f32,
                ))
            };

            na::Isometry3 {
                rotation,
                translation,
            }
        };

        // get size
        let Vector3 {
            x: sx,
            y: sy,
            z: sz,
        } = iobj.shape.dimensions;
        let sx = sx as f32;
        let sy = sy as f32;
        let sz = sz as f32;

        // compute bbox segments
        let segments: Vec<[na::Point3<f32>; 2]> =
            [[false, false], [false, true], [true, true], [true, false]]
                .iter()
                .circular_tuple_windows()
                .take(4)
                .flat_map(|(pbits, nbits)| {
                    let [px, py] = *pbits;
                    let [nx, ny] = *nbits;

                    [
                        [[px, py, false], [nx, ny, false]],
                        [[px, py, true], [nx, ny, true]],
                        [[px, py, false], [px, py, true]],
                    ]
                })
                .map(|[pbits, nbits]| {
                    let bit2one = |bit: bool| {
                        if bit {
                            1f32
                        } else {
                            -1f32
                        }
                    };
                    let to_point = |[xbit, ybit, zbit]: [bool; 3]| {
                        let point = na::Point3::new(
                            bit2one(xbit) * sx,
                            bit2one(ybit) * sy,
                            bit2one(zbit) * sz,
                        );
                        transform * point
                    };
                    let ppoint = to_point(pbits);
                    let npoint = to_point(nbits);
                    [ppoint, npoint]
                })
                .collect();

        let bbox_segments = {
            let color = na::Point3::new(1.0, 1.0, 0.0);
            ColoredSegmentSet { segments, color }
        };

        Object3D {
            bbox_segments,
            transform,
            size_xyz: [sx, sy, sz],
            classification: iobj.classification.clone(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, FromPrimitive)]
#[repr(usize)]
enum PointColorMode {
    Uniform = 0,
    Indensity,
    Distance,
    ObjectClass,
}

impl Default for PointColorMode {
    fn default() -> Self {
        Self::from_usize(0).unwrap()
    }
}

impl PointColorMode {
    pub fn next(&self) -> Self {
        match Self::from_usize(*self as usize + 1) {
            Some(next) => next,
            None => Self::default(),
        }
    }
}