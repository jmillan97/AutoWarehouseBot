# Camera And LiDAR Validation

This is the current bringup and validation path for higher camera FPS and
LiDAR-based obstacle data.

## Current Hardware Evidence

Camera:

```text
Microsoft LifeCam HD-3000
/dev/video0
YUYV 320x240 supports 10, 15, 20, and 30 fps
```

LiDAR:

```text
/dev/lidar   -> ttyUSB0
/dev/arduino -> ttyUSB1
```

The LiDAR responds on serial at `115200`, so the sensor and USB path are alive.

## Launch Defaults

`robot_bringup.launch.py` now defaults to:

```text
enable_lidar:=true
camera_width:=320
camera_height:=240
camera_framerate:=20.0
camera_pixel_format:=yuyv2rgb
```

Use `enable_lidar:=false` for camera-only testing.

## Camera FPS Test Ladder

Run the persistent launch yourself on the Pi.

Start with 20 fps:

```bash
source ~/.ros_network_env
ros2 launch embedded robot_bringup.launch.py enable_lidar:=false camera_framerate:=20.0
```

In another Pi terminal:

```bash
source ~/.ros_network_env
ros2 topic hz /camera/image_raw
```

If stable, try 30 fps:

```bash
source ~/.ros_network_env
ros2 launch embedded robot_bringup.launch.py enable_lidar:=false camera_framerate:=30.0
```

Success means:

```text
/camera/image_raw publishes near the requested rate
usb_cam does not crash or print select timeout errors
WSL can receive /camera/image_raw/compressed
```

YOLO may run slower than the camera stream. Treat camera publish rate, WSL
receive rate, and YOLO/display rate as separate measurements.

## LiDAR Scan Validation

Run the persistent launch on the Pi:

```bash
source ~/.ros_network_env
ros2 launch embedded robot_bringup.launch.py
```

In another Pi terminal:

```bash
source ~/.ros_network_env
ros2 topic list
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo --once /scan
```

For obstacle avoidance, the useful raw data is the `sensor_msgs/msg/LaserScan`
message:

```text
angle_min / angle_max
angle_increment
range_min / range_max
ranges[]
```

Each finite value in `ranges[]` becomes a 2D point in the LiDAR frame:

```text
x = range * cos(angle)
y = range * sin(angle)
```

The operator console now converts `/scan` into a live top-down point view and
shows:

```text
moving average scan rate
finite point count
nearest obstacle distance
```

## WSL Topic Crossing

From WSL, open a shell and source the ROS environment before checking topics:

```bash
cd ~/warehouse_project
source ~/.ros_network_env
ros2 topic list
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo --once /scan
```

The helper script also sources the local environment:

```bash
cd ~/warehouse_project
./scripts/check_topics.sh --snapshot
```

Interpretation:

```text
/scan on Pi and WSL    -> DDS path is good
/scan on Pi only       -> FastDDS/network/firewall issue
/scan nowhere          -> LiDAR launch/driver issue
/scan exists no data   -> driver started but scan stream is unhealthy
```

If topics stop crossing after an IP change, rediscover:

```bash
ip route get <pi_wifi_ip>
```

Use the `src` address as the laptop peer in:

```text
etc/fastdds_config.xml
/etc/fastdds_config.xml on the Pi
```

## Point Map Versus Room Map

The live `/scan` point view is a robot-relative obstacle snapshot. It is enough
to verify obstacle data and feed Nav2 costmaps.

A room-scale map needs robot pose over time:

```text
/scan + /tf + /odom or /odom_filtered
```

That is the SLAM/RViz/Nav2 layer. For the current goal, first make `/scan`
reliable on WSL; then Nav2 local/global costmaps can consume it for obstacle
avoidance.
