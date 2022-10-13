use crate::{color_sampling::sample_rgb, message as msg};
use async_std::task::spawn_blocking;
use futures::prelude::*;
use itertools::chain;
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
use std::collections::HashMap;

/// Starts the Kiss3d GUI interface.
pub async fn start(stream: impl Stream<Item = msg::Kiss3dMessage> + Unpin + Send) {
    // Creates a channel.
    let (tx, rx) = flume::bounded(2);

    // Creates a future that forwards stream messages to the channel.
    let forward_future = stream.map(Ok).forward(tx.into_sink()).map(|_result| ());

    // Spawn a non-async thread that runs Kiss3d loops.
    let handle_future = spawn_blocking(move || {
        // Creates a window.
        let window = {
            let mut window = Window::new("demo");
            window.set_light(Light::StickToCamera);
            window
        };
        // Configure the spectator camera.
        let mut camera = ArcBall::new(
            na::Point3::new(0.0, -80.0, 32.0),
            na::Point3::new(0.0, 0.0, 0.0),
        );
        camera.set_up_axis(na::Vector3::new(0.0, 0.0, 1.0));

        // Initialize the state
        let state = State {
            points: vec![],
            rx,
            camera,
            point_color_mode: PointColorMode::default(),
        };

        // Run rendering loops. It repeated calls state.step() method.
        window.render_loop(state);
    });

    // Wait for all futures to finish.
    futures::join!(forward_future, handle_future);
}

struct State {
    point_color_mode: PointColorMode,
    points: Vec<ColoredPoint>,
    rx: flume::Receiver<msg::Kiss3dMessage>,
    camera: ArcBall,
}

impl State {
    // Process pending events.
    fn process_events(&mut self, window: &mut Window) {
        window.events().iter().for_each(|evt| {
            use WindowEvent as E;

            match evt.value {
                E::Key(key, action, mods) => {
                    self.process_key_event(key, action, mods);
                }
                _ => {}
            }
        });
    }

    // Process pending keyboard events.
    fn process_key_event(&mut self, key: Key, action: Action, mods: Modifiers) {
        use Action as A;
        use Key as K;
        use Modifiers as M;

        let control = !(mods & M::Control).is_empty();
        let shift = !(mods & M::Shift).is_empty();
        let super_ = !(mods & M::Super).is_empty();

        match (key, action, control, shift, super_) {
            (K::Tab, A::Press, false, false, false) => {
                self.point_color_mode = self.point_color_mode.next();
            }
            _ => {}
        }
    }

    // A callback method called when a message arrives.
    fn update_msg(&mut self, msg: msg::Kiss3dMessage) {
        let msg::Kiss3dMessage {
            points,
            kneron_assocs,
        } = msg;

        // Collect background points
        let background_points = points.flatten().map(|point: msg::ArcPoint| {
            let color = na::Point3::new(0.3, 0.3, 0.3);
            (point, color)
        });

        // Collect points that are inside at least one bbox
        let object_points = kneron_assocs
            .as_ref()
            .map(|assocs: &msg::ArcAssocVec| {
                assocs.iter().filter_map(|assoc: &msg::Association| {
                    let point: msg::ArcPoint = assoc.pcd_point.clone();
                    let rect: &msg::ArcRect = assoc.rect.as_ref()?;
                    let [r, g, b] = sample_rgb(rect);
                    let color = na::Point3::new(r as f32, g as f32, b as f32);
                    Some((point, color))
                })
            })
            .into_iter()
            .flatten();

        // Merge background and object points into a hash map, indexed
        // by pointer address of points.
        let points: HashMap<msg::ArcPoint, na::Point3<f32>> =
            chain!(background_points, object_points).collect();

        // Store points along with their colors
        self.points = points
            .into_iter()
            .map(|(point, color)| ColoredPoint {
                position: point.position,
                color,
            })
            .collect();
    }

    fn render(&self, window: &mut Window) {
        // Draw axis
        self.draw_axis(window);

        // Draw points
        self.points.iter().for_each(|point| {
            let ColoredPoint { position, color } = point;
            window.draw_point(position, color);
        });
    }

    // A helper method to draw XYZ axis.
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
    /// A function to process rendering steps.
    fn step(&mut self, window: &mut Window) {
        // Process events
        self.process_events(window);

        // Try to receive a message
        match self.rx.try_recv() {
            Ok(msg) => {
                // update GUI state
                self.update_msg(msg);
            }
            Err(flume::TryRecvError::Empty) => {
                // Fall-through if the channel is empty.
            }
            Err(flume::TryRecvError::Disconnected) => {
                // Close the window if the channel is closed.
                window.close();
                return;
            }
        }

        // Run rendering.
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

/// A point position with an RGB color.
struct ColoredPoint {
    pub position: na::Point3<f32>,
    pub color: na::Point3<f32>,
}

/// A enum value marking the current point coloring method.
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
