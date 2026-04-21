# Robot Calibration Notes

## 2026-04-20: IMU Stationary Baseline

- Log folder: `calibration_logs/20260420_212807_imu`
- Robot state: flat and unmoving on table.
- Operator notes: motors not powered; Pi and camera near the IMU.
- IMU sample count: 2498 over 53.92 s.
- Timestamp rate: 46.31 Hz.
- Acceleration magnitude mean: 9.733 m/s^2.
- Gyro means: x 0.000033 rad/s, y 0.000011 rad/s, z -0.000070 rad/s.
- Quaternion norm mean: 1.000000.
- Yaw mean/stddev while still: -19.22 deg / 0.39 deg.
- Result: IMU stationary baseline passed. Data is stable enough for next calibration steps.

## 2026-04-20: Distance Trial 1, Commanded 200 mm

- Log folder: `calibration_logs/20260420_214230_distance`
- Commanded distance: 200 mm.
- Measured physical distance: about 110 mm.
- Drift: slight left drift, a couple millimeters.
- Operator notes: motion felt too fast and jerky; robot was realigned afterward.
- Encoder ticks: left 0 -> 89, right 0 -> 79.
- Average tick delta: 84 ticks.
- Physical scale from this run: 110 mm / 84 ticks = 1.31 mm/tick.
- Odometry end pose: x 0.329 m, y -0.119 m, yaw -11.43 deg.
- Odometry reported travel: about 350 mm.
- IMU yaw: -29.27 deg -> -27.26 deg, delta about +2.01 deg.
- Interpretation: current odometry distance scale is too large, and the command motion is too jerky at the previous speed.
- Action taken: reduce/hardlock serial bridge command speed to 90 before repeating the 200 mm trial.

## 2026-04-20: Distance Trial 2, Commanded 200 mm, Speed 90

- Log folder: `calibration_logs/20260420_220438_distance`
- Commanded distance: 200 mm.
- Measured physical distance: 100 mm.
- Drift: 0 mm; operator noted no drift at this speed.
- Encoder ticks: start left missing, start right 0; end left 66, end right 71.
- Right tick delta: 71 ticks. Left tick end was 66 ticks, but left delta was not computed because the start sample missed.
- Odometry start pose: x -0.350 m, y -0.035 m, yaw 11.43 deg.
- Odometry end pose: x -0.065 m, y -0.009 m, yaw 17.14 deg.
- Odometry reported travel: about 286.6 mm.
- IMU yaw: -22.48 deg -> -23.99 deg, delta about -1.51 deg.
- Interpretation: speed 90 made the motion more predictable, but command distance still undershot by about 50%, and odometry still overestimated travel by about 2.87x compared with the tape measurement.
- Next action: repeat 200 mm at speed 90 for more samples before changing odometry constants; if repeats stay near 100 mm, tune the command tick target and odometry scale separately.

## 2026-04-20: Distance Trial 3, Commanded 200 mm, Speed 90

- Log folder: `calibration_logs/20260420_221019_distance`
- Commanded distance: 200 mm.
- Measured physical distance: 90 mm.
- Drift: 0 mm.
- Encoder ticks: left 66 -> 137, right 71 -> 143.
- Tick deltas: left 71, right 72.
- Average tick delta: 71.5 ticks.
- Physical scale from this run: 90 mm / 71.5 ticks = 1.26 mm/tick.
- Odometry start pose: x -0.065 m, y -0.009 m, yaw 17.14 deg.
- Odometry end pose: x 0.232 m, y 0.026 m, yaw 18.29 deg.
- Odometry reported travel: about 298.9 mm.
- IMU yaw: -21.86 deg -> -19.81 deg, delta about +2.05 deg.
- Interpretation: second clean 200 mm speed-90 test confirms the command undershoots by about 50-55%, while odometry overestimates physical travel by about 3.32x.

## 2026-04-20: Distance Trial 4, Commanded 400 mm, Speed 90

- Log folder: `calibration_logs/20260420_221218_distance`
- Commanded distance: 400 mm.
- Measured physical distance: 120 mm.
- Drift: 0 mm.
- Operator notes: doubling the command had some effect, but not proportional.
- Encoder ticks: left 137 -> 239, right 143 -> 251.
- Tick deltas: left 102, right 108.
- Average tick delta: 105 ticks.
- Physical scale from this run: 120 mm / 105 ticks = 1.14 mm/tick.
- Odometry start pose: x 0.232 m, y 0.026 m, yaw 18.29 deg.
- Odometry end pose: x 0.663 m, y 0.111 m, yaw 25.14 deg.
- Odometry reported travel: about 439.1 mm.
- IMU yaw: -20.24 deg -> -21.42 deg, delta about -1.18 deg.
- Interpretation: command distance response is not linear yet; 400 mm command produced only 120 mm physical travel. This points to the motion controller stopping from encoder targets that do not match physical distance, plus odometry scale still being too large.

## 2026-04-20: Distance Trial 5, Commanded 600 mm, Speed 90

- Log folder: `calibration_logs/20260420_221519_distance`
- Commanded distance: 600 mm.
- Measured physical distance: 180 mm.
- Drift: about 10 mm left.
- Encoder ticks: left 239 -> 388, right 251 -> 411.
- Tick deltas: left 149, right 160.
- Average tick delta: 154.5 ticks.
- Physical scale from this run: 180 mm / 154.5 ticks = 1.17 mm/tick.
- Odometry start pose: x 0.663 m, y 0.111 m, yaw 25.14 deg.
- Odometry end pose: x 1.262 m, y 0.350 m, yaw 37.71 deg.
- Odometry reported travel: about 645.2 mm.
- IMU yaw: -21.29 deg -> -23.37 deg, delta about -2.08 deg.
- Interpretation: 400 mm and 600 mm commands are consistent at about 30% of requested physical travel. Odometry still overestimates physical travel by about 3.58x.
- Next action: tune wheel model scale down by about 3.5x for odometry, and increase motion target ticks by about 3.3x for distance commands if using the existing tick-based stop controller.

## 2026-04-20: Odometry Scale Patch

- Applied wheel model update: `gear_ratio` from 30.0 to 108.0, keeping `encoder_cpr` at 2.0.
- Effective CPR changed from 60 ticks/rev to 216 ticks/rev.
- Expected effect: distance-per-tick decreases by 3.6x, which should bring `/odom` much closer to tape measurements.
- Expected movement effect: `/move_distance_mm` target ticks increase by 3.6x, which should correct the observed 30% physical travel response for 400 mm and 600 mm commands.
- Command speed remains hardlocked at 90.
- Next validation: repeat 200 mm and 400 mm distance trials after Pi pull/rebuild/relaunch.

## 2026-04-20: Post-Patch Distance Trial 6, Commanded 200 mm, Speed 90, Gear Ratio 108

- Log folder: `calibration_logs/20260420_223544_distance`
- Commanded distance: 200 mm.
- Measured physical distance: 210 mm.
- Drift: about 10 mm left.
- Encoder ticks: left 0 -> 182, right 0 -> 188.
- Tick deltas: left 182, right 188.
- Average tick delta: 185 ticks.
- Odometry start pose: x 0.000 m, y 0.000 m, yaw 0.00 deg.
- Odometry end pose: x 0.215 m, y -0.006 m, yaw 1.90 deg.
- Odometry reported travel: about 215.2 mm.
- IMU yaw: -23.12 deg -> -18.02 deg, delta about +5.11 deg.
- Interpretation: gear ratio 108 corrected distance scale very well for a 200 mm command. Odom reported 215.2 mm vs 210 mm measured.

## 2026-04-20: Post-Patch Distance Trial 7, Commanded 400 mm, Speed 90, Gear Ratio 108

- Log folder: `calibration_logs/20260420_223824_distance`
- Commanded distance: 400 mm.
- Measured physical distance: 405 mm.
- Drift: operator notes 55 mm left drift, although the structured drift field was entered as 0 mm.
- Encoder ticks: start left 182, start right 188; end tick snapshot was missing.
- Odometry start pose: x 0.215 m, y -0.006 m, yaw 1.90 deg.
- Odometry end pose: x 0.638 m, y 0.012 m, yaw 7.62 deg.
- Odometry reported travel: about 423.3 mm.
- IMU yaw: -22.82 deg -> -9.90 deg, delta about +12.93 deg.
- Interpretation: gear ratio 108 also corrected distance scale well for a 400 mm command. However, the 55 mm left drift and large IMU yaw change show heading/straightness still needs calibration or control correction.
- Next action: keep gear ratio 108 for distance scale, then investigate left/right drive asymmetry and rotation calibration.

## 2026-04-20: Post-Patch Distance Trial 8, Commanded 400 mm, Speed 90, Gear Ratio 108

- Log folder: `calibration_logs/20260420_225031_distance`
- Commanded distance: 400 mm.
- Measured physical distance: 405 mm.
- Drift: 35 mm left.
- Encoder ticks: left 0 -> 358, right 0 -> 375.
- Tick deltas: left 358, right 375.
- Average tick delta: 366.5 ticks.
- Odometry start pose: x -0.637 m, y 0.042 m, yaw -7.62 deg.
- Odometry end pose: x -0.214 m, y -0.008 m, yaw -2.22 deg.
- Odometry reported travel: about 426.2 mm.
- IMU yaw: -20.48 deg -> -8.74 deg, delta about +11.75 deg.
- Interpretation: distance scale remains excellent, but repeated 400 mm trials show consistent left drift and a significant positive yaw change. Left/right tick delta also shows right ticks greater than left ticks, matching a left-curving path.
- Next action: keep gear ratio 108 and tune straight-line asymmetry/heading correction.

## 2026-04-20: Rotation Trial 1, Commanded +90 deg, Speed 90, Gear Ratio 108

- Log folder: `calibration_logs/20260420_230025_rotation`
- Commanded rotation: +90 deg.
- Measured physical rotation: about 60 deg to the right.
- Operator note: unsure if right is expected direction; suggests separate rotation and forward movement parameters may be needed.
- Direction prompt was marked correct, but physical note says the robot turned right.
- Encoder ticks: left 358 -> 523, right 375 -> 232.
- Tick deltas: left +165, right -143.
- Odometry yaw: -2.22 deg -> -100.00 deg, delta about -97.78 deg.
- IMU yaw: -11.45 deg -> -16.67 deg, delta about -5.22 deg.
- Script suggested rotation scale: 1.5 based on physical 60 deg vs commanded 90 deg.
- Interpretation: rotation behavior is not calibrated and may have a sign convention mismatch. Wheel odom reports almost the intended magnitude but opposite/rightward sign; physical measurement says only 60 deg right; fused IMU yaw did not track the observed physical turn reliably during this rotation snapshot.
- Next action: treat rotation as a separate calibration path from forward distance. Verify expected sign convention for `/rotate_angle_deg`, then run +90 and -90 physical tests before patching wheel separation or rotation-specific target scaling.

## 2026-04-20: Rotation Trial 2, Commanded -90 deg, Speed 90, Gear Ratio 108

- Log folder: `calibration_logs/20260420_230440_rotation`
- Commanded rotation: -90 deg.
- Measured physical rotation: about 60 deg left.
- Encoder ticks: left 523 -> 361, right 232 -> 384.
- Tick deltas: left -162, right +152.
- Odometry yaw: -100.00 deg -> -0.32 deg, delta about +99.68 deg.
- IMU yaw: -13.51 deg -> -10.22 deg, delta about +3.29 deg.
- Script suggested rotation scale is not directly usable because measured angle was entered as positive 60 for a negative command. Physical magnitude still indicates about 1.5x target scaling is needed.
- Interpretation: rotation sign is physically opposite the ROS comment convention. `+90` produced right turn, `-90` produced left turn. Magnitude is also low: both +/-90 commands produced about 60 deg physical rotation.
- Next action: flip rotate command sign mapping so positive degrees means physical left/CCW, and apply a rotation-specific target scale of about 1.5 without changing forward distance scale.

## 2026-04-20: Rotation Control Patch

- Added `rotation_scale = 1.5` to the serial bridge motion model.
- Applied `rotation_scale` only to `/rotate_angle_deg`; forward `/move_distance_mm` remains unchanged.
- Flipped rotation legacy command mapping so positive ROS rotation should produce physical left/CCW.
- Kept `gear_ratio = 108.0` and `command_speed = 90`.
- Next validation: run +90 and -90 rotation trials; expected physical results are about 90 deg left for +90 and about 90 deg right for -90.
