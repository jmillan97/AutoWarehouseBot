#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/flash_arduino.sh [options]

Compile and upload firmware.ino to the robot Arduino.

Options:
  --port PATH        Serial port to upload to (default: /dev/arduino)
  --fqbn FQBN        Arduino board FQBN (default: arduino:avr:uno)
  --sketch FILE      Sketch file (default: firmware.ino)
  --install-cli      Install arduino-cli into ~/bin if it is missing
  --no-upload        Compile only
  -h, --help         Show this help

Environment overrides:
  ARDUINO_PORT       Same as --port
  ARDUINO_FQBN       Same as --fqbn
EOF
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
port="${ARDUINO_PORT:-/dev/arduino}"
fqbn="${ARDUINO_FQBN:-arduino:avr:uno}"
sketch="$repo_root/firmware.ino"
install_cli=0
upload=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      port="$2"
      shift 2
      ;;
    --fqbn)
      fqbn="$2"
      shift 2
      ;;
    --sketch)
      sketch="$2"
      shift 2
      ;;
    --install-cli)
      install_cli=1
      shift
      ;;
    --no-upload)
      upload=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$sketch" ]]; then
  echo "Sketch not found: $sketch" >&2
  exit 1
fi

if ! command -v arduino-cli >/dev/null 2>&1; then
  if [[ "$install_cli" -ne 1 ]]; then
    cat >&2 <<'EOF'
arduino-cli is not installed.
Run again with --install-cli, or install arduino-cli and retry.
EOF
    exit 1
  fi

  if ! command -v curl >/dev/null 2>&1; then
    echo "curl is required to install arduino-cli." >&2
    exit 1
  fi

  mkdir -p "$HOME/bin"
  tmp_install="$(mktemp)"
  trap 'rm -f "$tmp_install"; [[ -n "${tmp_dir:-}" ]] && rm -rf "$tmp_dir"' EXIT
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh -o "$tmp_install"
  BINDIR="$HOME/bin" sh "$tmp_install"
  export PATH="$HOME/bin:$PATH"
else
  trap '[[ -n "${tmp_dir:-}" ]] && rm -rf "$tmp_dir"' EXIT
fi

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "arduino-cli installation failed or is not on PATH." >&2
  exit 1
fi

tmp_dir="$(mktemp -d)"
sketch_dir="$tmp_dir/firmware"
mkdir -p "$sketch_dir"
cp "$sketch" "$sketch_dir/firmware.ino"

echo "Using arduino-cli: $(command -v arduino-cli)"
echo "Sketch: $sketch"
echo "Board:  $fqbn"
echo "Port:   $port"

arduino-cli config init --overwrite >/dev/null 2>&1 || true
if arduino-cli core list | awk '{print $1}' | grep -qx 'arduino:avr'; then
  echo "Core arduino:avr already installed."
else
  arduino-cli core update-index
  arduino-cli core install arduino:avr
fi

if arduino-cli lib list | awk '{print $1}' | grep -qx 'EnableInterrupt'; then
  echo "Library EnableInterrupt already installed."
else
  arduino-cli lib update-index
  arduino-cli lib install EnableInterrupt
fi
arduino-cli compile --fqbn "$fqbn" "$sketch_dir"

if [[ "$upload" -eq 1 ]]; then
  if [[ ! -e "$port" ]]; then
    echo "Upload port does not exist: $port" >&2
    exit 1
  fi
  arduino-cli upload -p "$port" --fqbn "$fqbn" "$sketch_dir"
  echo "Arduino flash complete."
else
  echo "Compile complete; upload skipped."
fi
