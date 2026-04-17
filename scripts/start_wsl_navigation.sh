#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "Stopping any old local navigation processes..."
stop_local_stack

echo "Starting WSL navigation stack..."
source_local_ros_env
cd "${LOCAL_WORKSPACE}"
exec ros2 launch navigation hardware.launch.py "use_rviz:=${USE_RVIZ_VALUE}"
