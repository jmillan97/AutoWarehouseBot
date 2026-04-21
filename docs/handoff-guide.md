# Handoff Guide

This guide is the quick-start reference for communicating with the Pi, running
the WSL tools, pushing/pulling code, and sending movement commands.

## Current Branch

Use:

```bash
ros2-wireless
```

## Machine Roles

- Raspberry Pi:
  Hardware bringup, Arduino bridge, camera, encoders, odometry, IMU.
- WSL/laptop:
  Operator console, YOLO, Nav2, EKF, high-level commands.

## SSH Into The Pi

Codex users should prefer the MCP workflow in
[`docs/codex-mcp-workflow.md`](codex-mcp-workflow.md). The current Windows SSH
aliases are `warehouse-pi` for local Wi-Fi and `warehouse-pi-tail` for
Tailscale fallback.

From Windows PowerShell:

```powershell
ssh warehouse-pi
```

From WSL:

```bash
ssh ece_441@104.194.124.29
```

If the Pi IP changes, update commands/scripts that use:

```bash
104.194.124.29
```

## Pi ROS Setup Syntax

After SSHing into the Pi, source ROS and the installed workspace:

```bash
cd /home/ece_441/AutoWarehouseBot/ros2_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash
source ~/.ros_network_env
```

Start Pi hardware bringup:

```bash
ros2 launch embedded robot_bringup.launch.py
```

LiDAR is enabled by default. Disable it for camera-only testing or if
`/dev/lidar` is disconnected:

```bash
ros2 launch embedded robot_bringup.launch.py enable_lidar:=false
```

Current bringup defaults enable LiDAR and run the camera at 320x240 YUYV,
20 fps. See [`docs/camera-lidar-validation.md`](camera-lidar-validation.md) for
the camera FPS ladder, `/scan` checks, and WSL topic crossing workflow.

## WSL ROS Setup Syntax

In WSL:

```bash
cd ~/warehouse_project/ros2_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash
source ~/.ros_network_env
```

Run WSL navigation stack:

```bash
ros2 launch navigation hardware.launch.py use_rviz:=false
```

Run WSL navigation with standalone YOLO node:

```bash
ros2 launch navigation hardware.launch.py use_rviz:=false use_yolo:=true
```

## Operator Console

Preferred operator workflow:

```bash
cd ~/warehouse_project
source ~/.ros_network_env
python3 scripts/operator_console.py
```

The operator console:

- reads `/camera/image_raw/compressed`
- displays the camera feed
- runs YOLO locally if available
- sends movement commands to ROS

Useful environment overrides:

```bash
OPERATOR_USE_YOLO=0 python3 scripts/operator_console.py
OPERATOR_YOLO_MODEL=/home/felix/warehouse_project/yolov8n.pt python3 scripts/operator_console.py
OPERATOR_CAMERA_TOPIC=/camera/image_raw/compressed python3 scripts/operator_console.py
```

## Movement Command Syntax

From the operator console:

```text
forward 3 ft
back 250 mm
move 1.2 m
left 90 deg
right 45 deg
rotate -30 deg
move_mm 500
rotate_deg -90
help
```

Raw ROS topic syntax from Pi or WSL:

```bash
source ~/.ros_network_env
ros2 topic pub --once /move_distance_mm std_msgs/msg/Int32 "{data: 500}"
ros2 topic pub --once /move_distance_mm std_msgs/msg/Int32 "{data: -300}"
ros2 topic pub --once /rotate_angle_deg std_msgs/msg/Int32 "{data: 90}"
ros2 topic pub --once /rotate_angle_deg std_msgs/msg/Int32 "{data: -45}"
```

Conventions:

- `/move_distance_mm`
  - positive = forward
  - negative = backward
- `/rotate_angle_deg`
  - positive = counterclockwise
  - negative = clockwise

## Camera Checks

On WSL:

```bash
source ~/.ros_network_env
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

Viewer only:

```bash
cd ~/warehouse_project
source ~/.ros_network_env
python3 scripts/view_camera.py
```

Current Pi camera settings live in:

```bash
ros2_ws/src/embedded/launch/robot_bringup.launch.py
```

Current expected settings:

```text
image_width: 320
image_height: 240
framerate: 10.0
pixel_format: yuyv2rgb
```

## Git Push Syntax

Windows-side git has been more reliable for pushing this repo than WSL git.

From Windows PowerShell:

```powershell
git -C \\wsl.localhost\Ubuntu-24.04\home\felix\warehouse_project status -sb
git -C \\wsl.localhost\Ubuntu-24.04\home\felix\warehouse_project push origin ros2-wireless
```

Normal WSL git syntax also works for local commits:

```bash
cd ~/warehouse_project
git status --short
git add <files>
git commit -m "Short message"
```

## Pull Code Onto The Pi

SSH into the Pi, then:

```bash
cd /home/ece_441/AutoWarehouseBot
git pull origin ros2-wireless
git log --oneline -1
```

If Pi-side ROS package files changed, rebuild on the Pi:

```bash
cd /home/ece_441/AutoWarehouseBot/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build --packages-select embedded
source install/setup.bash
```

If only WSL scripts/docs/navigation launch files changed, the Pi usually does
not need a rebuild.

## Build On WSL

Build changed WSL packages:

```bash
cd ~/warehouse_project/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build --packages-select navigation perception_yolo
source install/setup.bash
```

Build Pi-side package locally:

```bash
cd ~/warehouse_project/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build --packages-select embedded
source install/setup.bash
```

## Stop / Restart

Stop processes manually with `Ctrl+C` in their terminal.

Use helper script from WSL:

```bash
cd ~/warehouse_project
export PI_PASSWORD='group4pi'
./scripts/stop_robot_stack.sh
```

If camera gets stuck:

1. stop Pi bringup
2. unplug/replug USB camera
3. relaunch Pi bringup
4. reboot Pi only if USB camera stays wedged

## Known Issues

- `usb_cam` may still crash with `Select timeout, exiting...`.
- Missing camera calibration file warning is not the main issue.
- KDL root-link inertia warning is harmless for current testing.
- Windows PowerShell profile currently prints noisy errors; these do not affect
  ROS commands.
- `yolov8n.pt` is intentionally not committed.
