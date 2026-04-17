# Startup Scripts

These scripts are committed to the repo so they can be pushed on a branch and
shared with the team.

They are meant to be run from WSL.

## What They Do

- `scripts/start_pi_robot.sh`
  Starts the Raspberry Pi hardware bringup over SSH
- `scripts/start_wsl_navigation.sh`
  Starts the WSL navigation stack locally
- `scripts/start_robot_stack.sh`
  Starts the Pi bringup first, then starts the WSL navigation stack
- `scripts/stop_robot_stack.sh`
  Kills the known ROS processes on both WSL and the Pi so you do not have to
  hunt down hung processes manually

## Prerequisites

Before using the scripts, both machines should already have:

- `~/.ros_network_env`
- `~/.bashrc` sourcing `~/.ros_network_env`

See the local troubleshooting docs for the exact file contents.

The WSL machine must also be able to SSH into the Pi.

## SSH Authentication

The scripts support either:

1. normal SSH keys
2. password-based SSH using `PI_PASSWORD`

Example with password auth:

```bash
export PI_PASSWORD='group4pi'
./scripts/start_robot_stack.sh
```

If you use SSH keys, you do not need `PI_PASSWORD`.

## Machine-Specific Variables

You can override these if needed:

```bash
export PI_USER=ece_441
export PI_HOST=104.194.126.139
export PI_WORKSPACE=/home/ece_441/AutoWarehouseBot/ros2_ws
export LOCAL_WORKSPACE=/home/felix/warehouse_project/ros2_ws
export USE_RVIZ_VALUE=false
```

## Typical Usage

### Start everything from WSL

```bash
export PI_PASSWORD='group4pi'
./scripts/start_robot_stack.sh
```

That will:

1. stop any old Pi bringup processes
2. start `robot_bringup.launch.py` on the Pi
3. wait a few seconds
4. stop any old WSL navigation processes
5. start `hardware.launch.py` on WSL

### Start only the Pi hardware side

```bash
export PI_PASSWORD='group4pi'
./scripts/start_pi_robot.sh
```

### Start only the WSL navigation side

```bash
./scripts/start_wsl_navigation.sh
```

### Stop everything cleanly

```bash
export PI_PASSWORD='group4pi'
./scripts/stop_robot_stack.sh
```

## What Gets Killed

The stop script intentionally kills the usual bringup processes so stale copies
do not accumulate.

On the Pi it targets things like:

- `robot_bringup.launch.py`
- `usb_cam_node_exe`
- `imu_node`
- `serial_bridge`
- `wheel_odometry`
- `ekf_node`
- `rplidar_composition`

On WSL it targets things like:

- `hardware.launch.py`
- `rviz2`
- `component_container_isolated`
- Nav2 servers
- `robot_state_publisher`
- local `ekf_node`

## Why This Helps

During debugging, duplicate Pi bringups and duplicate sensor nodes caused a lot
of confusion. These scripts are meant to keep startup and shutdown consistent so
you do not have to manually clean up zombie processes.
