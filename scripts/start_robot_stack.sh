#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting remote Pi hardware bringup first..."
"${SCRIPT_DIR}/start_pi_robot.sh"

echo
echo "Waiting a moment before starting local navigation..."
sleep 4

echo
echo "Starting local WSL navigation..."
exec "${SCRIPT_DIR}/start_wsl_navigation.sh"
