#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

HOOK_LINE='[ -f ~/.ros_network_env ] && source ~/.ros_network_env'

ensure_local_env_file() {
  cat > "${HOME}/.ros_network_env" <<EOF
source /opt/ros/kilted/setup.bash
if [ -f ${LOCAL_WORKSPACE}/install/setup.bash ]; then
  source ${LOCAL_WORKSPACE}/install/setup.bash
fi
export FASTDDS_DEFAULT_PROFILES_FILE=${LOCAL_FASTDDS}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_VALUE}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY_VALUE}
EOF
}

ensure_local_hook() {
  touch "${HOME}/.bashrc"
  if ! grep -qxF "${HOOK_LINE}" "${HOME}/.bashrc"; then
    printf '\n%s\n' "${HOOK_LINE}" >> "${HOME}/.bashrc"
  fi
}

ensure_pi_env_and_hook() {
  remote_bash "
cat > '/home/${PI_USER}/.ros_network_env' <<'EOF'
source /opt/ros/kilted/setup.bash
if [ -f '${PI_WORKSPACE}/install/setup.bash' ]; then
  source '${PI_WORKSPACE}/install/setup.bash'
fi
export FASTDDS_DEFAULT_PROFILES_FILE='${PI_FASTDDS}'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_VALUE}'
export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_VALUE}'
export ROS_LOCALHOST_ONLY='${ROS_LOCALHOST_ONLY_VALUE}'
EOF

touch '/home/${PI_USER}/.bashrc'
if ! grep -qxF '${HOOK_LINE}' '/home/${PI_USER}/.bashrc'; then
  printf '\n%s\n' '${HOOK_LINE}' >> '/home/${PI_USER}/.bashrc'
fi
"
}

print_local_values() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${HOME}/.ros_network_env"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
  echo "WSL_FASTDDS=${FASTDDS_DEFAULT_PROFILES_FILE}"
  echo "WSL_RMW=${RMW_IMPLEMENTATION}"
  echo "WSL_DOMAIN=${ROS_DOMAIN_ID}"
  echo "WSL_LOCALHOST=${ROS_LOCALHOST_ONLY}"
}

print_pi_values() {
  remote_bash "
set +u
source '/home/${PI_USER}/.ros_network_env'
set -u
echo PI_FASTDDS=\${FASTDDS_DEFAULT_PROFILES_FILE}
echo PI_RMW=\${RMW_IMPLEMENTATION}
echo PI_DOMAIN=\${ROS_DOMAIN_ID}
echo PI_LOCALHOST=\${ROS_LOCALHOST_ONLY}
"
}

echo "Writing WSL ROS env file..."
ensure_local_env_file
echo "Ensuring WSL bashrc hook..."
ensure_local_hook

echo "Writing Pi ROS env file + hook..."
ensure_pi_env_and_hook

echo "Verifying active values (WSL)..."
print_local_values

echo "Verifying active values (Pi)..."
print_pi_values

echo "Done."
