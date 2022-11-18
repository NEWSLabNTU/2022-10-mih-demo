# Architecture

![](legend.dot.png)

![](architecture.dot.png)

## Nodes

- `velodyne_driver_node`, `velodyne_convert_node`:

  These nodes provided by
  [`velodyne`](https://github.com/ros-drivers/velodyne) package scans
  and converts the point cloud stream from a lidar sensor.


- `kneron_bbox_server_node`

  This node reads bounding box stream from the Kneron camera.

- `v4l2_node`

  This node reads video stream from the Otobrite camera.

- `newslab_fuse_demo`

  This node gathers input sensor data and fuse them together, and
  shows the results in prompted windows.

- `det_conv_node`

  This node demonstrates message type conversions to Autoware
  messages.
