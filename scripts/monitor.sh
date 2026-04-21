#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "Running topic snapshot..."
"${SCRIPT_DIR}/check_topics.sh" --snapshot

echo "Opening camera feed window..."
exec python3 "${SCRIPT_DIR}/view_camera.py"
