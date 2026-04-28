#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PI_USER="${PI_USER:-ece_441}"
PI_HOST="${PI_HOST:-104.194.122.149}"
PI_WORKSPACE="${PI_WORKSPACE:-/home/${PI_USER}/AutoWarehouseBot/ros2_ws}"
PI_FASTDDS="${PI_FASTDDS:-/etc/fastdds_config.xml}"

LOCAL_WORKSPACE="${LOCAL_WORKSPACE:-${REPO_ROOT}/ros2_ws}"
LOCAL_FASTDDS="${LOCAL_FASTDDS:-${REPO_ROOT}/etc/fastdds_config.xml}"

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-42}"
ROS_LOCALHOST_ONLY_VALUE="${ROS_LOCALHOST_ONLY_VALUE:-0}"
RMW_IMPLEMENTATION_VALUE="${RMW_IMPLEMENTATION_VALUE:-rmw_fastrtps_cpp}"
USE_RVIZ_VALUE="${USE_RVIZ_VALUE:-false}"

PI_LOG_PATH="${PI_LOG_PATH:-/home/${PI_USER}/pi_robot_bringup.log}"

remote_ssh() {
  if [[ -n "${PI_PASSWORD:-}" ]]; then
    sshpass -p "${PI_PASSWORD}" ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_HOST}" "$@"
  else
    ssh -o StrictHostKeyChecking=no "${PI_USER}@${PI_HOST}" "$@"
  fi
}

remote_bash() {
  remote_ssh "bash -s" <<< "$1"
}

source_local_ros_env() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  if [[ -f "${HOME}/.ros_network_env" ]]; then
    # shellcheck disable=SC1090
    source "${HOME}/.ros_network_env"
  else
    source /opt/ros/kilted/setup.bash
    if [[ -f "${LOCAL_WORKSPACE}/install/setup.bash" ]]; then
      # shellcheck disable=SC1091
      source "${LOCAL_WORKSPACE}/install/setup.bash"
    fi
    export FASTDDS_DEFAULT_PROFILES_FILE="${LOCAL_FASTDDS}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION_VALUE}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY_VALUE}"
  fi

  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

stop_local_stack() {
  local patterns=(
    "hardware.launch.py"
    "rviz2"
    "component_container_isolated"
    "planner_server"
    "controller_server"
    "bt_navigator"
    "smoother_server"
    "behavior_server"
    "velocity_smoother"
    "waypoint_follower"
    "collision_monitor"
    "map_server"
    "amcl"
    "lifecycle_manager"
    "robot_state_publisher"
    "ekf_node"
  )

  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
}

stop_pi_stack() {
  local remote_script
  remote_script=$(cat <<'EOF'
patterns=(
  "robot_bringup.launch.py"
  "usb_cam_node_exe"
  "imu_node"
  "serial_bridge"
  "wheel_odometry"
  "ekf_node"
  "rplidar_composition"
  "usb_cam_debug"
  "demo_nodes_cpp talker"
)

for pattern in "${patterns[@]}"; do
  pkill -9 -f "${pattern}" 2>/dev/null || true
done
EOF
)

  remote_bash "${remote_script}"
}
