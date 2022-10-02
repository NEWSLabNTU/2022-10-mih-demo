# MIH 2022 Demo Repository

This repositroy stores demo programs for MIH 2022 exhibition.

## Prerequisites

**ROS 2 Galactic** is required to build the source code. For Ubuntu
users, follow the official installation guide for Debian/ubuntu
[](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). Arch
Linux users can install `ros2-galactic` on AUR. Other Linux users can
refer to the "fat" archive installtion guide
([link](https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Install-Binary.html)).

**Rust toolchain** is required to compile Rust programs. Please
install the toolchain using the command on
[rustup.rs](https://rustup.rs/).

## Build

Run `make` to build the project.

Follow the steps below if you prefer to build the project manually.

```bash
## Pull dependent ROS repos
mkdir src
vcs import repos < dependencies.repos
vcs pull repos < dependencies.repos

## Source ROS setup script
source /opt/ros/galactic/setup.sh

## Build dependent ROS packages
cd repos && colcon build && source install/setup.sh

## Build the project
cargo build --all-targets --release
```

## Usage

### 2D Detection Server for Kneron Camera

To retrieve detected bounding boxes from a Kneron camera, connect the
Kneron board via Ethernet cable. Set the network static address to
172.23.230.N/24 (pick a N here). Start the ROS node by:

```bash
./target/release/kneron-bbox-server-node
```

### Visualizer

To start the visualizer to show live video and bounding boxes,

```bash
./target/release/viz
```

To change the input topic names,

```bash
./target/release/viz --img-topic IMG_TOPIC --det-topic DET_TOPIC --pcd-topic PCD_TOPIC
```
