#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/drive_test.sh forward [seconds]
  ./scripts/drive_test.sh backward [seconds]
  ./scripts/drive_test.sh left [seconds]
  ./scripts/drive_test.sh right [seconds]
  ./scripts/drive_test.sh stop
  ./scripts/drive_test.sh custom <lin_x> <ang_z> [seconds]

Notes:
  - Run from WSL after Pi bringup is running.
  - Script auto-sends a stop command after timed motions.
EOF
}

if [[ "${1:-}" == "" ]]; then
  usage
  exit 1
fi

CMD="${1}"
DURATION="${2:-1.5}"
RATE=10
LIN_X=0.0
ANG_Z=0.0

case "${CMD}" in
  forward)
    LIN_X=0.15
    ANG_Z=0.0
    ;;
  backward)
    LIN_X=-0.12
    ANG_Z=0.0
    ;;
  left)
    LIN_X=0.0
    ANG_Z=0.6
    ;;
  right)
    LIN_X=0.0
    ANG_Z=-0.6
    ;;
  stop)
    LIN_X=0.0
    ANG_Z=0.0
    DURATION=0
    ;;
  custom)
    if [[ "${3:-}" == "" ]]; then
      usage
      exit 1
    fi
    LIN_X="${2}"
    ANG_Z="${3}"
    DURATION="${4:-1.5}"
    ;;
  *)
    usage
    exit 1
    ;;
esac

source_local_ros_env

publish_twist() {
  local lx="$1"
  local az="$2"
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${lx}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${az}}}" >/dev/null
}

if [[ "${DURATION}" == "0" ]]; then
  echo "Sending stop command..."
  publish_twist 0.0 0.0
  exit 0
fi

echo "Publishing /cmd_vel lin_x=${LIN_X} ang_z=${ANG_Z} for ${DURATION}s..."
timeout "${DURATION}" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: ${LIN_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANG_Z}}}" -r "${RATE}" >/dev/null || true
echo "Sending stop command..."
publish_twist 0.0 0.0
echo "Done."
