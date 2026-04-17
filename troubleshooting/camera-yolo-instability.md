# Camera + YOLO Instability Notes

This note summarizes the current state of the camera and YOLO pipeline, what is
working, what is failing, and the most likely causes.

## Current Working Pieces

- Robot motion control is working again through the ROS topic interface.
- Pi camera bringup works with:
  - `image_width: 320`
  - `image_height: 240`
  - `framerate: 10.0`
  - `pixel_format: mjpeg2rgb`
- WSL can display the plain camera feed in a local viewer.
- WSL YOLO package scaffolding exists and builds successfully.
- WSL YOLO model file exists locally:
  - `/home/felix/warehouse_project/yolov8n.pt`
- WSL YOLO node can start and load the model.
- A local YOLO overlay window appeared at least once, which proves that the
  general architecture works.

## Current Failure Pattern

The main failure is instability, not total breakage.

Observed behavior:

- Sometimes `/camera/image_raw` reaches WSL and displays correctly.
- Sometimes WSL sees the ROS topic graph, but actual image payloads do not
  arrive.
- The YOLO node can start and subscribe, but if camera frames do not reach WSL
  consistently, no detections or overlay updates appear.
- At least once, the Pi camera node crashed after YOLO display briefly worked:
  - `usb_cam_node_exe`
  - `Select timeout, exiting...`
  - process died and respawned

## What This Most Likely Means

The likely issue is not the YOLO logic itself.

The likely weak point is the camera producer and/or image transport path:

- Pi camera node instability under load
- WSL image transport instability over DDS/Wi-Fi
- Pi thermal stress
- Pi USB/camera stability problems
- power quality issues affecting the Pi or USB camera

## Strong Clues Collected

### 1. YOLO itself is not the main problem

Evidence:

- `perception_yolo` package builds
- model loads
- detector starts
- a local YOLO overlay window appeared once

That means the inference path is fundamentally valid.

### 2. The camera path is the unstable component

Evidence:

- plain camera viewing worked without YOLO
- later, camera topics were visible in ROS graph but did not reliably deliver
  frames to WSL
- Pi-side `usb_cam` later crashed with:
  - `Select timeout, exiting...`

### 3. Heavy ROS/image traffic likely makes things worse

Likely contributing load:

- camera stream
- YOLO inference
- Nav2 stack
- EKF
- DDS transport over Wi-Fi

### 4. Power may be a real contributing factor

The whole system was connected to battery power while charging.

That may cause:

- unstable voltage
- noise on the power rails
- USB instability
- Pi camera glitches
- random webcam timeout behavior

This is especially plausible because earlier debugging already uncovered real
power problems on the robot side.

### 5. Thermal load may also be contributing

The Pi heatsink became very hot to the touch and the system became sluggish.

That suggests:

- sustained high Pi load
- possible thermal throttling
- less margin for stable camera handling

## Current Best Architecture

For now, the correct split is:

### Pi

- publish camera
- publish lidar
- publish IMU
- publish encoder/odometry
- run serial motor bridge
- execute simple movement commands

### WSL

- run YOLO
- run visualization
- run Nav2
- run heavy perception logic
- send only simple control outputs back to the robot

This means the YOLO overlay should be treated primarily as a **local WSL debug
window**, not as a heavy annotated image topic that must be shipped around ROS.

## Current YOLO Strategy

The YOLO node was refactored so that:

- it subscribes to `/camera/image_raw`
- it opens a local OpenCV overlay window on WSL
- it publishes lightweight outputs:
  - `/perception/yolo/detections`
  - `/perception/yolo/people`
- it does **not** need to publish an annotated image topic by default

This is the better short-term design because it reduces ROS image traffic.

## Most Likely Root Causes

In order of suspicion:

1. Pi camera node instability (`usb_cam`) under current load
2. WSL DDS image transport instability for large image topics over Wi-Fi
3. Pi thermal throttling / overheating
4. power instability from battery + charging setup
5. USB camera sensitivity to power/noise/load conditions

## Best Next Steps

### Immediate stabilization

- let the Pi cool down fully before more testing
- reboot both Pi and laptop/WSL before the next serious round
- start with the minimum stack only

### Minimum restart test

1. Pi:
   - `robot_bringup.launch.py`
2. WSL:
   - test plain camera only
3. WSL:
   - test YOLO alone without the full Nav2 stack
4. Only then:
   - add Nav2 back in

### Suggested camera reductions if instability continues

- reduce to `160x120`
- reduce to `5 FPS`
- keep `pixel_format: mjpeg2rgb`

This is not ideal for quality, but it is a reasonable stability-first step.

### Power / hardware sanity checks

- test with the robot stationary
- avoid mixing “charging battery” and “stress-testing camera/compute” if
  possible
- use the cleanest power path available
- check USB camera seating/cable stability

## Summary

The system is closer than it first appears:

- camera works
- WSL display works
- YOLO code works
- local overlay concept works

The remaining problem is stability in the camera production/transport path,
most likely aggravated by Pi load, heat, Wi-Fi transport, and possibly shared
power conditions.
