use anyhow::Result;
use kneron_bbox_server::{BoundingBox, Server};
use r2r::{
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::Pose2D,
    std_msgs::msg::Header,
    vision_msgs::msg::{BoundingBox2D, BoundingBox2DArray},
    Context, Node, QosProfile,
};
use std::time::{SystemTime, UNIX_EPOCH};

fn main() -> Result<()> {
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "kneron_bbox_publisher", "namespace")?;
    let server = Server::new()?;
    let publisher = node.create_publisher::<BoundingBox2DArray>("TOPIC", QosProfile::default())?;

    server
        .enumerate()
        .try_for_each(|(frame_id, result)| -> Result<()> {
            let result = result?;

            let boxes: Vec<_> = result
                .boxes()
                .iter()
                .map(|bbox| {
                    let BoundingBox {
                        x1,
                        y1,
                        x2,
                        y2,
                        score: _,
                        class_num: _,
                    } = *bbox;
                    let size_x = x2 as f64 - x1 as f64;
                    let size_y = y2 as f64 - y1 as f64;
                    let cx = x1 as f64 + size_x / 2.0;
                    let cy = y1 as f64 + size_y / 2.0;

                    BoundingBox2D {
                        center: Pose2D {
                            x: cx,
                            y: cy,
                            theta: 0.0,
                        },
                        size_x,
                        size_y,
                    }
                })
                .collect();
            let systime = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
            let msg = BoundingBox2DArray {
                header: Header {
                    stamp: Time {
                        sec: systime.as_secs() as i32,
                        nanosec: systime.subsec_nanos() as u32,
                    },
                    frame_id: frame_id.to_string(),
                },
                boxes,
            };

            publisher.publish(&msg)?;

            Ok(())
        })?;

    Ok(())
}
