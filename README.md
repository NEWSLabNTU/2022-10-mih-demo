# MIH 2022 Demo Repository

This repositroy stores demo programs for MIH 2022 exhibition.

## Prerequisites

**Rust toolchain** is required to compile Rust programs. Please
install the toolchain using the command on
[rustup.rs](https://rustup.rs/).


To retrieve detected bounding boxes from a Kneron camera, connect the
Kneron board via Ethernet cable. Set the network static address to
172.23.230.N/24 (pick a N here). Start the ROS node by:


## Usage

### Case 1: Run the server individually

Start the server using the commands below. The server prints the data
as soon as it retrieves messages from the Kneron camera.

```bash
cd crates/kneron-bbox-server
cargo run --release --example capture
```

### Case 2: Run the server as a ROS Node

**ROS 2 Galactic** is required to build the source code. For Ubuntu
users, follow the official installation guide for Debian/ubuntu
[](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). Arch
Linux users can install `ros2-galactic` on AUR. Other Linux users can
refer to the "fat" archive installtion guide
([link](https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Install-Binary.html)).


To build the ROS node, go to the toplevel directory and run the commands.

```bash
## Source ROS setup script
source /opt/ros/galactic/setup.sh

make build_ros_dependencies
make build
```

To start the ROS node,

```bash
./target/release/kneron_bbox_server_node
```
