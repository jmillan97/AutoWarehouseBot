# Firmware + Bridge Protocol

This project uses the **known-good legacy Arduino firmware** for motor control.
Movement primitives are implemented in `serial_bridge` on the Pi.

## Current Architecture

- Arduino firmware accepts legacy serial commands:
  - `speed:<0-255>`
  - `w`, `a`, `s`, `d`, `q`, `e`, `x`, `r`
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
4. Repeatedly sends legacy motion command (`w/s/q/e`) to keep watchdog alive
5. Monitors encoder delta from `E:left,right`
6. Sends `x` stop when target ticks reached (or timeout)

This keeps firmware unchanged while giving deterministic movement commands from
ROS.

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
