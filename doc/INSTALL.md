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



## Prepare The Repository and Dependencies

This section describes essential steps before building this project.

1. Prepare the project repository.

  ```bash
  git clone https://github.com/NEWSLabNTU/v2x-demo-2022.git
  cd v2x-demo-2022
  ```

2. Source the `setup.bash` for ROS. This step is required whenever you
   start a new shell.

  ```bash
  source /opt/ros/galactic/setup.bash
  ```

## Build

### Method 1: Using Makefile (Recommended)

Build this project using `make`. It will take several minutes.

```bash
make
```

The build is successful if it shows the following message.

```
Summary: 17 packages finished [2min 28s]
```

### Method 2: Build Manually


Pull dependent ROS packages using `vcs`.

```bash
vcs import src < deps.repos
```

Build the whole project using `colcon`. It will take several minutes.

```bash
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
```

## Run Lidar-Camera Fusing Demo

Source the generated `setup.bash` file. You may source `setup.zsh`
instead if you're using zsh.

```bash
source install/setup.bash
```

Execute the script to start the demo.


```bash
./scripts/start.sh
```
