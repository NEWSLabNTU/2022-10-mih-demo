//! Provides a ROS node that starts a server listening to detection
//! messages from a Kneron camera.

use anyhow::Result;
use clap::Parser;
use kneron_bbox_server::{BoundingBox, Server};
use r2r::{
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::{Point, Pose, Pose2D, PoseWithCovariance, Quaternion},
    std_msgs::msg::Header,
    vision_msgs::msg::{
        BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose,
    },
    Context, Node, QosProfile,
};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Parser)]
struct Opts {
    #[clap(long, default_value = "kneron_detecion")]
    pub topic: String,
    #[clap(long, default_value = "/")]
    pub namespace: String,
}

fn main() -> Result<()> {
    let opts = Opts::parse();

    // Start the server listening to a Kneron camera.
    let server = Server::new()?;

    // Start a ROS node
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, env!("CARGO_PKG_NAME"), &opts.namespace)?;

    // Create a ROS publisher.
    let publisher =
        node.create_publisher::<Detection2DArray>(&opts.topic, QosProfile::default())?;

    server
        .enumerate()
        .try_for_each(|(frame_id, result)| -> Result<()> {
            let result = result?;
            let header = {
                let systime = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
                Header {
                    stamp: Time {
                        sec: systime.as_secs() as i32,
                        nanosec: systime.subsec_nanos() as u32,
                    },
                    frame_id: frame_id.to_string(),
                }
            };

            let detections: Vec<_> = result
                .boxes()
                .iter()
                .map(|ibbox| {
                    let BoundingBox {
                        x1,
                        y1,
                        x2,
                        y2,
                        score,
                        class_num,
                    } = *ibbox;
                    let size_x = x2 as f64 - x1 as f64;
                    let size_y = y2 as f64 - y1 as f64;
                    let cx = x1 as f64 + size_x / 2.0;
                    let cy = y1 as f64 + size_y / 2.0;

                    Detection2D {
                        header: header.clone(),
                        results: vec![ObjectHypothesisWithPose {
                            hypothesis: ObjectHypothesis {
                                class_id: class_num.to_string(),
                                score: score as f64,
                            },
                            pose: PoseWithCovariance {
                                pose: Pose {
                                    position: Point {
                                        x: cx,
                                        y: cy,
                                        z: 0.0,
                                    },
                                    orientation: Quaternion {
                                        x: 0.0,
                                        y: 0.0,
                                        z: 0.0,
                                        w: 1.0,
                                    },
                                },
                                covariance: identity_covariance(),
                            },
                        }],
                        bbox: BoundingBox2D {
                            center: Pose2D {
                                x: cx,
                                y: cy,
                                theta: 0.0,
                            },
                            size_x,
                            size_y,
                        },
                        id: "".to_string(),
                    }
                })
                .collect();
            let msg = Detection2DArray { header, detections };

            publisher.publish(&msg)?;

            Ok(())
        })?;

    Ok(())
}

/// Generated a flattened 6x6 identity matrix
fn identity_covariance() -> Vec<f64> {
    let mut matrix = vec![0f64; 36];
    (0..6).for_each(|idx| {
        matrix[idx * 6 + idx] = 1.0;
    });
    matrix
}
