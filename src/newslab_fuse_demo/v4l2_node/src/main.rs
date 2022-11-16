mod config;

use anyhow::{bail, ensure, Result};
use clap::Parser;
use config::Config;
use r2r::{
    builtin_interfaces::msg::Time, sensor_msgs::msg::Image, std_msgs::msg::Header, Context, Node,
    QosProfile,
};
use rscam::Camera;
use std::{
    fs,
    sync::{
        mpsc::{sync_channel, SyncSender},
        Arc,
    },
    thread::spawn,
    time::{Duration, SystemTime},
};

#[derive(Debug, Clone, Parser)]
struct Opts {
    #[clap(long)]
    pub config: String,
}

fn main() -> Result<()> {
    let opts = Opts::parse();
    let config: Config = {
        let text = fs::read_to_string(&opts.config)?;
        json5::from_str(&text)?
    };
    let config = Arc::new(config);

    // Start a ROS node
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, env!("CARGO_PKG_NAME"), &config.namespace)?;
    // Create a ROS publisher.
    let publisher = node.create_publisher::<Image>(&config.topic, QosProfile::default())?;

    // Create a channel
    let (tx, rx) = sync_channel(8);

    let capture_handle = spawn({
        let config = config.clone();
        move || run_camera_capture(&config, tx)
    });
    let publish_handle = spawn(move || {
        while let Ok(image) = rx.recv() {
            publisher.publish(&image)?;
        }
        anyhow::Ok(())
    });
    let spin_handle = spawn(move || -> ! {
        loop {
            node.spin_once(Duration::from_millis(10));
        }
    });

    capture_handle.join().unwrap()?;
    publish_handle.join().unwrap()?;
    spin_handle.join().unwrap();
}

fn run_camera_capture(config: &Config, tx: SyncSender<Image>) -> Result<()> {
    let Config {
        resolution,
        interval,
        ref format,
        ..
    } = *config;
    let (width, height) = resolution;

    let pixel_step = match format.as_str() {
        "UYVY" => 2,
        "RGB8" => 3,
        _ => bail!("unsupported format {}", format),
    };

    // Start the camera
    let mut camera = Camera::new(&config.video_device)?;
    camera.start(&rscam::Config {
        interval,
        resolution,
        format: format.as_bytes(),
        ..Default::default()
    })?;

    let mut frame_id_iter = 0..;

    loop {
        let frame = camera.capture()?;

        let sys_time = SystemTime::now();
        let unix_time = sys_time.duration_since(SystemTime::UNIX_EPOCH).unwrap();

        let row_step = pixel_step * width;
        let num_bytes = row_step * height;
        let bytes: &[u8] = &*frame;
        let is_bigendian = if cfg!(target_endian = "big") { 1 } else { 0 };

        ensure!(
            frame.resolution == config.resolution,
            "resolution mismatches"
        );
        ensure!(
            frame.format == config.format.as_bytes(),
            "format mismatches"
        );
        ensure!(
            bytes.len() == num_bytes as usize,
            "byte array length mismatches"
        );

        let frame_id = frame_id_iter.next().unwrap();
        let image = Image {
            header: Header {
                stamp: Time {
                    sec: unix_time.as_secs() as i32,
                    nanosec: unix_time.subsec_nanos(),
                },
                frame_id: frame_id.to_string(),
            },
            height,
            width,
            encoding: format.clone(),
            is_bigendian,
            step: row_step,
            data: bytes.to_vec(),
        };

        let ok = tx.send(image).is_ok();
        if !ok {
            break;
        }
    }

    Ok(())
}
