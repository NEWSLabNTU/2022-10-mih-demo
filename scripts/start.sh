#!/usr/bin/env bash
set -e

script_dir=$(realpath -- "$(dirname -- "${BASH_SOURCE[0]}")")
cd "$script_dir"

source /opt/ros/galactic/setup.bash
source repos/install/setup.bash

# cargo build --release --all-targets


parallel -j0 --lb <<EOF
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
ros2 launch velodyne_pointcloud velodyne_convert_node-VLP32C-launch.py
./target/release/kneron_bbox_server_node
./target/release/v4l2_node --config crates/v4l2_node/config/example.json5
env DISPLAY=:0 ./target/release/camera_viz --config crates/camera_viz/config/example.json5
EOF
