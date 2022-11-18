#!/usr/bin/env bash
set -e

script_dir=$(realpath -- "$(dirname -- "${BASH_SOURCE[0]}")")
cd "$script_dir"

if [[ ! -v ROS_DISTRO ]]; then
    echo 'Did you source install/setup.{sh,bash,zsh,..} ?' >&2
    exit 1
fi

parallel -j0 --lb <<EOF
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py
ros2 launch velodyne_pointcloud velodyne_convert_node-VLP32C-launch.py
ros2 run kneron_bbox_server_node kneron_bbox_server_node
ros2 run otobrite_v4l2_node otobrite_v4l2_node -- --config $script_dir/../src/newslab_fuse_demo/otobrite_v4l2_node/config/example.json5
ros2 run newslab_fuse_demo newslab_fuse_demo -- --config $script_dir/../src/newslab_fuse_demo/newslab_fuse_demo/config/example.json5
EOF
