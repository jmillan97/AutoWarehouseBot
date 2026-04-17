#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "Stopping any old Pi bringup processes..."
stop_pi_stack

echo "Starting Pi hardware bringup on ${PI_USER}@${PI_HOST}..."
remote_bash "
export FASTDDS_DEFAULT_PROFILES_FILE='${PI_FASTDDS}'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_VALUE}'
export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
source /opt/ros/kilted/setup.bash
if [ -f '${PI_WORKSPACE}/install/setup.bash' ]; then
  source '${PI_WORKSPACE}/install/setup.bash'
fi
nohup ros2 launch embedded robot_bringup.launch.py > '${PI_LOG_PATH}' 2>&1 < /dev/null &
echo PI_PID=\$!
echo PI_LOG='${PI_LOG_PATH}'
"
