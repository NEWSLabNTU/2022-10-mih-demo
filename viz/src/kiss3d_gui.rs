use anyhow::Result;
use async_std::task::{spawn_blocking, JoinHandle};
use kiss3d::{
    camera::{ArcBall, Camera},
    light::Light,
    nalgebra as na,
    planar_camera::PlanarCamera,
    post_processing::PostProcessingEffect,
    window::Window,
};
use r2r::{
    autoware_auto_perception_msgs::msg::DetectedObjects, geometry_msgs::msg::Point32,
    sensor_msgs::msg::PointCloud,
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
            rx,
            camera,
        };
        window.render_loop(state);
        anyhow::Ok(())
    });

    (handle, tx)
}

struct State {
    points: Vec<Point>,
    rx: flume::Receiver<Message>,
    camera: ArcBall,
}

impl State {
    fn update_msg(&mut self, msg: Message) {
        match msg {
            Message::PointCloud(pcd) => self.update_point_cloud(pcd),
            Message::DetectedObjects(objs) => self.update_aw_objs(objs),
        }
    }

    fn update_point_cloud(&mut self, pcd: PointCloud) {
        self.points = pcd
            .points
            .iter()
            .map(|ipt| {
                let Point32 { x, y, z } = *ipt;
                let position = na::Point3::new(x, y, z);
                let color = na::Point3::new(1.0, 1.0, 1.0);
                Point { position, color }
            })
            .collect();
    }

    fn update_aw_objs(&self, objs: DetectedObjects) {
        objs.objects.iter().map(|obj| {
            // obj.shape.polygon;
            // TODO
        });
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

        // Draw axis
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

        // Draw points
        self.points.iter().for_each(|point| {
            let Point { position, color } = point;
            window.draw_point(position, color);
        });
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

struct Point {
    pub position: na::Point3<f32>,
    pub color: na::Point3<f32>,
}

pub enum Message {
    PointCloud(PointCloud),
    DetectedObjects(DetectedObjects),
}

impl From<DetectedObjects> for Message {
    fn from(v: DetectedObjects) -> Self {
        Self::DetectedObjects(v)
    }
}

impl From<PointCloud> for Message {
    fn from(v: PointCloud) -> Self {
        Self::PointCloud(v)
    }
}
