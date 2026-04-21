#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

SNAPSHOT_ONLY=0
if [[ "${1:-}" == "--snapshot" ]]; then
  SNAPSHOT_ONLY=1
fi

KEY_TOPICS=(
  /camera/image_raw
  /odom
  /scan
  /imu/data
)

HZ_DURATION=5

print_header() {
  echo ""
  echo "========================================"
  echo "  $1"
  echo "========================================"
}

# ── Phase A: topic list from both sides ──────────────────────────────────────

print_header "[PI] Topics published"
remote_bash "
  source /opt/ros/kilted/setup.bash
  if [ -f /home/ece_441/AutoWarehouseBot/ros2_ws/install/setup.bash ]; then
    source /home/ece_441/AutoWarehouseBot/ros2_ws/install/setup.bash
  fi
  export FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds_config.xml
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0
  ros2 topic list
" 2>/dev/null

print_header "[WSL] Topics visible"
source_local_ros_env
ros2 topic list 2>/dev/null

# ── Diff: topics Pi has that WSL can't see ────────────────────────────────────

print_header "Key topic presence check"
for topic in "${KEY_TOPICS[@]}"; do
  pub_count=$(ros2 topic info "${topic}" 2>/dev/null | grep -oP 'Publisher count: \K[0-9]+' || echo 0)
  if [[ "${pub_count}" -gt 0 ]]; then
    echo "  [OK]  ${topic}  (${pub_count} publisher)"
  elif ros2 topic info "${topic}" &>/dev/null; then
    echo "  [!!]  ${topic}  -- visible but NO active publisher (node may have crashed)"
  else
    echo "  [!!]  ${topic}  -- NOT visible on WSL (DDS/network problem)"
  fi
done

if [[ "${SNAPSHOT_ONLY}" -eq 1 ]]; then
  echo ""
  exit 0
fi

# ── Phase B: rate check on both sides ─────────────────────────────────────────

print_header "[PI] Topic rates (${HZ_DURATION}s sample)"
remote_bash "
  source /opt/ros/kilted/setup.bash
  if [ -f /home/ece_441/AutoWarehouseBot/ros2_ws/install/setup.bash ]; then
    source /home/ece_441/AutoWarehouseBot/ros2_ws/install/setup.bash
  fi
  export FASTDDS_DEFAULT_PROFILES_FILE=/etc/fastdds_config.xml
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0
  for topic in /camera/image_raw /odom /scan /imu/data; do
    echo -n \"  \${topic}: \"
    timeout ${HZ_DURATION} ros2 topic hz \${topic} 2>/dev/null | grep -m1 'average rate' || echo 'no data'
  done
" 2>/dev/null &
PI_RATE_PID=$!

print_header "[WSL] Topic rates (${HZ_DURATION}s sample)"
for topic in /camera/image_raw /odom /scan /imu/data; do
  echo -n "  ${topic}: "
  timeout ${HZ_DURATION} ros2 topic hz "${topic}" 2>/dev/null | grep -m1 'average rate' || echo 'no data'
done

wait "${PI_RATE_PID}" 2>/dev/null || true

echo ""
