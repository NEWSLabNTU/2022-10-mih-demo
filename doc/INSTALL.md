# Installation

## Prerequisites

This project have several Rust packages integrated into ROS 2 colcon
build system. Rust toolchain and colcon plugins are required to build
this project.

- **Rust toolchain** provides the compiler and `cargo` package manager
  for Rust packages.

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  ```

- **cargo-ament-build** is an extension to `cargo` that generates
  ROS-compatible installation. This command installs the patched
  version.

  ```bash
  cargo install --git https://github.com/jerry73204/cargo-ament-build.git
  ```

- **colcon-ros-cargo** is an extension to `colcon` to identify Rust
  packages maintained by `cargo` package manager. This command
  installs the patched version.

  ```bash
  pip3 install git+https://github.com/jerry73204/colcon-ros-cargo.git@merge-colcon-cargo
  ```

- **ROS 2** galactic or newer is required to build this
  project. Galactic version was tested for this project. Here is the
  [installation
  guide](https://docs.ros.org/en/galactic/Installation.html).



## Prepare

This section describes essential steps before building this project.

1. Prepare the project repository.

  ```bash
  git clone https://github.com/NEWSLabNTU/v2x-demo-2022.git
  cd v2x-demo-2022
  ```

2. Source the `setup.bash` for ROS. This step is required whenever you
   start a new shell.

  ```bash
  source /opt/ros/galactic/setup.zsh
  ```

3. List available packages using `colcon list` in project
   directory. Verify that the list includes the following
   packages. The package type must be `ament_cargo`.

  ```
  kneron_bbox_server_node src/newslab_fuse_demo/kneron_bbox_server_node   (ament_cargo)
  newslab_fuse_demo       src/newslab_fuse_demo/newslab_fuse_demo (ament_cargo)
  v4l2_node       src/newslab_fuse_demo/v4l2_node (ament_cargo)
  vision_to_autoware_conv_node    src/newslab_fuse_demo/vision_to_autoware_conv_node      (ament_cargo)
  ```

## Build

Build the whole project using `colcon`. It will take several minutes.

```bash
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
```

## Run Lidar-Camera Fusing Demo

**The section is outdated** is yet to be rewritten. The new
instructions will come soon.

### (A) Publish Velodyne LiDAR point clouds

Edit the configuration file. Specify the LiDAR device IP address in
the `device_ip` field.

```
./repos/velodyne/velodyne_driver/config/VLP32C-velodyne_driver_node-params.yaml
```

Run the following commands in two separate terminals.

```bash
# Terminal 1
ros2 launch velodyne_driver velodyne_driver_node-VLP32C-launch.py

# Terminal 2
ros2 launch velodyne_pointcloud velodyne_convert_node-VLP32C-launch.py
```


### (B) Run 2D detection server for Kneron camera

To retrieve detected bounding boxes from a Kneron camera, connect the
Kneron board via Ethernet cable. Set the network static address to
172.23.230.N/24 (pick a N here). Start the ROS node by:

```bash
./target/release/kneron_bbox_server_node
```


### (C) Capture images from Otobrite camera

Set the video device path in the config file
`crates/v4l2_node/config/example.json5`. For example, `/dev/video0`.

Start the image capturing node.

```bash
cargo run --bin v4l2_node --release -- --config crates/v4l2_node/config/example.json5
```

### (D) Run the visualizer for Otobrite and Kneron cameras

Modify input topic names in the `crates/camera_viz/config/example.json5`
configuration file. Then,

```bash
./target/release/camera_viz --config crataes/camera_viz/config/example.json5
```

Before running the visualizer, you may run (A), (B) and (C) first.

### (E) Run `lidar_centerpoint`

Go check [wiki](https://newslabn.csie.ntu.edu.tw:3000/en/wayside-team/notes/2022-09-22_run-autoware-lidar_centerpoint) to setup autoware and Rviz2 environment. And follow the commend to run lidar_centerpoint.

### (F) Run `det_conv_node`

```bash
./target/release/det_conv_node
```
