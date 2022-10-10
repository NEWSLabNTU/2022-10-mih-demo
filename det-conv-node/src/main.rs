use anyhow::Result;
use async_std::task::spawn_blocking;
use clap::Parser;
use futures::{
    future::{FutureExt as _, TryFutureExt as _},
    stream::{StreamExt as _, TryStreamExt as _},
};
use r2r::{
    autoware_auto_perception_msgs::msg::{
        DetectedObject, DetectedObjectKinematics, DetectedObjects, ObjectClassification, Shape,
    },
    geometry_msgs::msg::{Polygon, TwistWithCovariance, Vector3},
    log_warn,
    vision_msgs::msg::{
        BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesis, ObjectHypothesisWithPose,
    },
    Context, Node, QosProfile,
};
use std::time::Duration;

#[derive(Parser)]
struct Opts {
    #[clap(long)]
    pub input_topic: String,
    #[clap(long)]
    pub output_topic: Option<String>,
    #[clap(long, default_value = "/")]
    pub namespace: String,
}

#[async_std::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();

    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "det_conv_node", &opts.namespace)?;

    let subscriber =
        node.subscribe::<Detection2DArray>(&opts.input_topic, QosProfile::default())?;
    let publisher = opts
        .output_topic
        .as_ref()
        .map(|topic| node.create_publisher::<DetectedObjects>(topic, QosProfile::default()))
        .transpose()?;

    let stream = subscriber
        .map(|det| {
            let Detection2DArray { header, detections } = det;

            let objects = detections
                .into_iter()
                .map(|det| {
                    let Detection2D { results, bbox, .. } = det;

                    let kinematics = {
                        let pose = results.get(0).map(|result| &result.pose);
                        let has_position_covariance = pose.is_some();
                        let pose_with_covariance = pose.cloned().unwrap_or_default();

                        DetectedObjectKinematics {
                            pose_with_covariance,
                            has_position_covariance,
                            orientation_availability: 0,
                            twist_with_covariance: TwistWithCovariance::default(),
                            has_twist: false,
                            has_twist_covariance: false,
                        }
                    };

                    let shape = {
                        let BoundingBox2D { size_x, size_y, .. } = bbox;
                        Shape {
                            type_: 0, // BOUNDING_BOX
                            footprint: Polygon::default(),
                            dimensions: Vector3 {
                                x: size_x,
                                y: size_y,
                                z: 0.0,
                            },
                        }
                    };

                    let classification: Vec<_> = results
                        .into_iter()
                        .map(|result| {
                            let ObjectHypothesisWithPose {
                                hypothesis: ObjectHypothesis { class_id, score },
                                ..
                            } = result;

                            let label: u8 = match class_id.parse() {
                                Ok(val) => val,
                                Err(_) => {
                                    log_warn!(
                                        env!("CARGO_PKG_NAME"),
                                        "class_id '{}' is not a number. Assume label=0.",
                                        class_id
                                    );
                                    0
                                }
                            };

                            ObjectClassification {
                                label,
                                probability: score as f32,
                            }
                        })
                        .collect();

                    DetectedObject {
                        existence_probability: 1.0,
                        classification,
                        kinematics,
                        shape,
                    }
                })
                .collect();

            DetectedObjects { header, objects }
        })
        .inspect(|objects| {
            println!("{:?}", objects);
        });

    let conv_future = match publisher {
        Some(publisher) => stream
            .map(anyhow::Ok)
            .try_fold(publisher, |publisher, msg| async move {
                publisher.publish(&msg)?;
                Ok(publisher)
            })
            .map_ok(|_publisher| ())
            .boxed(),
        None => stream
            .map(anyhow::Ok)
            .try_for_each(|_| async move { Ok(()) })
            .boxed(),
    };

    let spin_future = spawn_blocking(move || loop {
        node.spin_once(Duration::from_millis(100));
    });

    futures::try_join!(conv_future, spin_future.map(anyhow::Ok))?;

    Ok(())
}
