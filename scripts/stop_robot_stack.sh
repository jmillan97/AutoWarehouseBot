#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

echo "Stopping local WSL navigation processes..."
stop_local_stack

echo "Stopping remote Pi hardware processes on ${PI_USER}@${PI_HOST}..."
stop_pi_stack

echo "Done."
