# Firmware + Bridge Protocol

This project uses the **known-good legacy Arduino firmware** for motor control.
Movement primitives are implemented in `serial_bridge` on the Pi.

## Current Architecture

- Arduino firmware accepts legacy serial commands:
  - `speed:<0-255>`
  - `w`, `a`, `s`, `d`, `q`, `e`, `x`, `r`
  - `drive_lr:<left_pwm>,<right_pwm>`
- Arduino publishes encoder lines:
  - `E:<left_ticks>,<right_ticks>`
- `serial_bridge` translates ROS movement topics into those legacy commands.

## ROS Movement Topics (authoritative API)

- `/move_distance_mm` (`std_msgs/msg/Int32`)
  - signed millimeters: `+` forward, `-` backward
- `/rotate_angle_deg` (`std_msgs/msg/Int32`)
  - signed degrees: `+` CCW, `-` CW

## Bridge Translation Logic

`serial_bridge` behavior:

1. Receives `/move_distance_mm` or `/rotate_angle_deg`
2. Converts target (mm/deg) to encoder ticks using wheel model params
3. Sends one `speed:<value>` command
4. For linear moves, sends `drive_lr:<left_pwm>,<right_pwm>` with encoder tick balancing
5. For rotation moves, repeatedly sends legacy motion command (`q/e`) to keep watchdog alive
6. Monitors encoder delta from `E:left,right`
7. Sends `x` stop when target ticks reached (or timeout)

This keeps legacy firmware commands available while allowing the Pi to correct
straight-line left/right drift through independent side PWM.

## Independent Side Drive

`drive_lr:<left_pwm>,<right_pwm>` accepts signed PWM values in `-255..255`.

- Positive values drive that side forward.
- Negative values drive that side backward.
- `drive_lr:0,0` stops both sides.

Examples:

```text
drive_lr:90,90
drive_lr:80,95
drive_lr:-90,90
```

## Flashing Firmware

Use the repo flasher when changing `firmware.ino`:

```bash
cd /home/ece_441/AutoWarehouseBot
./scripts/flash_arduino.sh --port /dev/arduino --fqbn arduino:avr:uno
```

First-time setup on a fresh Pi installs Arduino CLI, the AVR core, and the
`EnableInterrupt` library:

```bash
./scripts/flash_arduino.sh --install-cli --port /dev/arduino --fqbn arduino:avr:uno
```

Defaults:

- port: `/dev/arduino`
- board: `arduino:avr:uno`
- sketch: repo-root `firmware.ino`

The script creates a temporary Arduino sketch folder before compiling because
Arduino tooling expects the `.ino` filename to match its parent folder. It also
skips reinstalling the AVR core and `EnableInterrupt` library once they already
exist.

Before flashing, stop Pi bringup or at least the serial bridge so `/dev/arduino`
is not busy:

```bash
pkill -f serial_bridge_py
```

After a successful flash, a quick serial sanity check should show:

```text
ACK: SYSTEM_READY
E:0,0
```

## Why This Model

- Preserves known-good motor behavior in firmware
- Avoids re-debugging low-level motor logic
- Keeps movement API clean on ROS side (distance + angle only)

## Test Commands

```bash
source ~/.ros_network_env
ros2 topic pub --once /move_distance_mm std_msgs/msg/Int32 "{data: 500}"
ros2 topic pub --once /move_distance_mm std_msgs/msg/Int32 "{data: -300}"
ros2 topic pub --once /rotate_angle_deg std_msgs/msg/Int32 "{data: 90}"
ros2 topic pub --once /rotate_angle_deg std_msgs/msg/Int32 "{data: -45}"
```
