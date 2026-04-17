# Firmware Protocol Notes

This file documents the Arduino firmware command protocol and current ROS2
movement interfaces.

## Why We Changed It

The original firmware accepted manual commands only and not structured
distance/angle commands from ROS.

## Original Firmware Behavior

- Accepted:
  - `speed:<0-255>`
  - `w`, `a`, `s`, `d`, `q`, `e`, `x`, `r`
- Published encoder feedback:
  - `E:<left_ticks>,<right_ticks>`
- Did **not** parse ROS-style velocity commands.

## Current Firmware Behavior (Movement API)

Still supports all original commands, plus:

- `move_mm:<signed_mm>`
  - New exact-distance mode using encoder ticks
  - `move_mm:500` moves forward ~500 mm
  - `move_mm:-250` moves backward ~250 mm
  - Stops automatically when target ticks reached
  - Emits ACK lines:
    - `ACK: MOVE_START ...`
    - `ACK: MOVE_DONE`
    - `ACK: MOVE_TIMEOUT`

- `rotate_deg:<signed_deg>`
  - Exact-angle mode using encoder ticks + wheel separation model
  - `rotate_deg:90` rotates CCW about +90 deg
  - `rotate_deg:-45` rotates CW about -45 deg
  - Emits ACK lines:
    - `ACK: ROTATE_START ...`
    - `ACK: ROTATE_DONE`
    - `ACK: ROTATE_TIMEOUT`

## ROS Bridge Mapping

`serial_bridge` mappings now:

- `/move_distance_mm` (`std_msgs/msg/Int32`) -> `move_mm:<signed_mm>`
- `/rotate_angle_deg` (`std_msgs/msg/Int32`) -> `rotate_deg:<signed_deg>`
- Arduino encoder lines `E:l,r` -> `/left_ticks`, `/right_ticks`

## How To Use Exact Distance From ROS

On Pi or WSL (with ROS env sourced and bridge running), linear move:

```bash
ros2 topic pub --once /move_distance_mm std_msgs/msg/Int32 "{data: 500}"
```

Backward:

```bash
ros2 topic pub --once /move_distance_mm std_msgs/msg/Int32 "{data: -300}"
```

Rotate:

```bash
ros2 topic pub --once /rotate_angle_deg std_msgs/msg/Int32 "{data: 90}"
ros2 topic pub --once /rotate_angle_deg std_msgs/msg/Int32 "{data: -45}"
```

## Tuning Notes

- Distance accuracy depends on:
  - wheel radius
  - encoder CPR
  - gear ratio
  - wheel slip / floor
- Constants are currently aligned with `wheel_odometry` defaults.
- Recalibrate with measured runs (e.g., command 1000 mm, measure actual).
