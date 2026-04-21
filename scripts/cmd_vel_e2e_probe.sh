#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

source_local_ros_env

echo "Starting Pi-side /cmd_vel listener..."
remote_bash "
export FASTDDS_DEFAULT_PROFILES_FILE='${PI_FASTDDS}'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_VALUE}'
export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
source /opt/ros/kilted/setup.bash
timeout 12 ros2 topic echo /cmd_vel --once > /tmp/cmd_vel_once.log 2>&1 &
echo LISTENER_PID=\$!
"

sleep 2

echo "Publishing forward command from WSL..."
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 >/dev/null || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null || true

sleep 2

echo "Pi-side listener output:"
remote_bash "cat /tmp/cmd_vel_once.log || true"
