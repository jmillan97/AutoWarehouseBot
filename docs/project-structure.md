# Project Structure

Short map of the current repo layout after the ROS 2, camera, and operator
console changes.

## Top-Level Files

- `firmware.ino`
  Arduino firmware for motor pins, encoders, serial commands, exact distance
  moves, and exact rotation moves.
- `scripts/`
  WSL-side helper scripts for starting/stopping robot processes, checking
  topics, viewing the camera, and running the operator console.
- `docs/`
  Working documentation for startup, firmware protocol, structure, and handoff.
- `troubleshooting/`
  Debug notes for known failure modes. Some files in this folder are force-added
  because the folder is ignored by default.
- `ros2_ws/`
  Main ROS 2 workspace.

## ROS 2 Packages

- `ros2_ws/src/embedded`
  Raspberry Pi hardware package.
  Starts the serial bridge, wheel odometry, IMU, robot state publisher, and
  camera. LiDAR is disabled by default until it is plugged back in. EKF is no
  longer launched on the Pi.
- `ros2_ws/src/navigation`
  WSL/offboard navigation package.
  Owns `hardware.launch.py`, Nav2 launch wiring, and the offboard EKF.
- `ros2_ws/src/perception_yolo`
  WSL YOLO ROS node package.
  Kept for standalone YOLO experiments, though the operator console now also has
  local YOLO overlay support.
- `ros2_ws/src/perception`
  Existing C++ perception stubs/fake sensor helpers.
- `ros2_ws/src/description`
  Robot URDF/xacro and description launch.
- `ros2_ws/src/summon` and `ros2_ws/src/summon_msgs`
  Summon/landmark-related Python package and messages.

## Current Runtime Split

- Raspberry Pi:
  - Arduino serial bridge
  - encoder tick handling
  - wheel odometry
  - IMU publishing
  - camera publishing
  - optional LiDAR when `enable_lidar:=true`
- WSL/laptop:
  - operator console
  - YOLO overlay / inference
  - Nav2
  - EKF
  - high-level planning and UI

## Important Current Topics

- `/camera/image_raw`
  Raw camera frames from the Pi.
- `/camera/image_raw/compressed`
  Compressed camera transport used by WSL viewers and the operator console.
- `/move_distance_mm`
  Signed `std_msgs/msg/Int32`; positive forward, negative backward.
- `/rotate_angle_deg`
  Signed `std_msgs/msg/Int32`; positive CCW, negative CW.
- `/left_ticks`, `/right_ticks`
  Encoder tick outputs from the serial bridge.
- `/odom`
  Wheel odometry from the Pi.
- `/odom_filtered`
  Offboard EKF output from WSL navigation.

## Current Operator UI

- `scripts/operator_console.py`
  WSL GUI with:
  - camera feed
  - local YOLO overlay if `ultralytics` and `yolov8n.pt` are available
  - movement buttons
  - text command entry for distance/angle movement

