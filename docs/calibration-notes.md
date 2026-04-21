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

## 2026-04-20: Post-Patch Rotation Trial 3, Commanded +90 deg

- Log folder: `calibration_logs/20260420_231618_rotation`
- Commanded rotation: +90 deg.
- Measured physical rotation: about 95 deg left.
- Encoder ticks: left 0 -> -236, right 0 -> 227.
- Tick deltas: left -236, right +227.
- Odometry yaw: 0.00 deg -> 146.98 deg.
- IMU yaw: -12.63 deg -> -10.79 deg, delta about +1.84 deg.
- Interpretation: physical sign is now correct and magnitude is close. Odom yaw overestimates rotation substantially, while IMU yaw snapshots still under-report rotation.

## 2026-04-20: Post-Patch Rotation Trial 4, Commanded -90 deg

- Log folder: `calibration_logs/20260420_231835_rotation`
- Commanded rotation: -90 deg.
- Measured physical rotation: about 92 deg right.
- Encoder ticks: left -236 -> 1, right 227 -> 12.
- Tick deltas: left +237, right -215.
- Odometry yaw: 146.98 deg -> 3.49 deg, delta about -143.49 deg.
- IMU yaw: -11.63 deg -> -29.98 deg, delta about -18.36 deg.
- Interpretation: physical sign is correct and magnitude is close. Rotation control is now usable for +/-90 commands, but odometry yaw overestimates rotation and IMU yaw remains unreliable for these snapshot-only rotation measurements.
- Next action: keep `rotation_scale = 1.5` for command behavior, then separately calibrate wheel odom yaw by tuning `wheel_separation` or choosing whether EKF should trust IMU/wheel yaw.

## 2026-04-20: Post-Patch Rotation Trial 5, Commanded +180 deg

- Log folder: `calibration_logs/20260420_232323_rotation`
- Commanded rotation: +180 deg.
- Measured physical rotation: about 181 deg left.
- Operator notes: basically perfect.
- Encoder ticks: left 1 -> -448, right 12 -> 435.
- Tick deltas: left -449, right +423.
- Odometry yaw: 3.49 deg -> -79.68 deg. The raw snapshot delta is misleading because yaw wraps across the +/-180 range during a large turn.
- IMU yaw: -11.74 deg -> -31.41 deg, delta about -19.67 deg.
- Interpretation: command-side rotation behavior is now excellent for 90 and 180 deg turns. Do not change `rotation_scale` based on this run. Odom and IMU yaw snapshots are not reliable enough by themselves for large rotation validation without unwrapping/continuous capture.
- Next action: keep command calibration fixed and improve rotation logging/analysis before tuning wheel odom yaw.

## 2026-04-20: Manual Square Path Test, 400 mm Sides

- Sequence: four 400 mm forward moves and four +90 deg left turns.
- Distance logs used:
  - `calibration_logs/20260420_234329_distance`
  - `calibration_logs/20260420_234924_distance`
  - `calibration_logs/20260420_235241_distance`
  - `calibration_logs/20260420_235619_distance`
- Rotation logs used:
  - `calibration_logs/20260420_234649_rotation`
  - `calibration_logs/20260420_235134_rotation`
  - `calibration_logs/20260420_235424_rotation`
  - `calibration_logs/20260420_235726_rotation`
- Forward segments were recorded as 400 mm each.
- Left drift was entered as about 20 mm for each forward segment.
- Rotation segments were recorded as about 93 deg each.
- Final physical position: X changed from 0 cm to 8 cm, so X error is about +8 cm. Y changed from 11 cm to 29 cm, so Y error is about +18 cm.
- Final physical heading: about 60 deg left of starting heading.
- Interpretation: individual distance and turn commands are usable, but open-loop square driving still accumulates too much heading/pose error. The repeated left drift and large final heading error point toward needing heading correction during forward motion and better continuous yaw logging.
- Next action: implement a single `square` calibration command that streams `/odom`, `/imu/data`, and ticks continuously across the full path, then use that data to tune heading correction/EKF rather than relying on separate start/end snapshots.

## 2026-04-21: Lightweight Square Drift Test, 400 mm Sides

- Log folder: `calibration_logs/20260421_001504_square`
- Sequence: four 400 mm forward moves and four +90 deg left turns.
- Pause after each segment: 6.0 s.
- Physical final pose note: left corner tire was used as the reference point; final position ended about 13 cm right and 38 cm forward of start.
- Interpreted final physical error: X about +380 mm forward, Y about -130 mm right, heading about +60 deg left.
- Script numeric final fields were left as 0, so the notes field is the trusted physical measurement.
- Odom start pose: x 0.657 m, y 0.284 m, yaw -108.89 deg.
- Odom end pose: x 0.823 m, y -0.032 m, yaw 143.17 deg.
- Odom delta: x +165.9 mm, y -315.7 mm, yaw -107.94 deg using wrapped start/end yaw.
- IMU yaw: -30.21 deg -> -31.38 deg, delta about -1.17 deg.
- Interpretation: repeating the square with automatic pauses still leaves a large final pose error. Since individual 400 mm and 90/180 deg commands are accurate, open-loop square driving is dominated by systematic drift/heading accumulation rather than command scale.
- Next action: add heading correction during forward motion or apply an open-loop drift compensation term; EKF alone cannot fix the physical path error.

## 2026-04-21: Independent Side Drive Patch

- Added Arduino serial command: `drive_lr:<left_pwm>,<right_pwm>`.
- Signed PWM convention: positive drives that side forward; negative drives that side backward.
- Legacy commands remain available: `w`, `s`, `q`, `e`, `x`, `speed:`.
- Updated `serial_bridge_py` to use `drive_lr` for linear `/move_distance_mm` commands only.
- Rotation commands remain on the calibrated legacy `q/e` path.
- Added encoder tick balancing during linear moves:
  - `tick_error = left_delta - right_delta`
  - `correction = linear_balance_kp * tick_error`
  - `left_pwm = command_speed - correction`
  - `right_pwm = command_speed + correction`
- Initial `linear_balance_kp`: 0.4.
- Next validation: upload firmware, restart Pi bringup, then run 400 mm distance and square tests to see whether left drift decreases.

## 2026-04-21: Post-Flash Independent Drive Distance Trial

- Log folder: `calibration_logs/20260421_012229_distance`
- Commanded distance: 200 mm.
- Measured physical distance: 240 mm.
- Physical drift: about 20 mm left.
- Encoder ticks: left 0 -> 195, right 0 -> 209.
- Tick deltas: left +195, right +209.
- Odom distance: about 235.0 mm.
- Odom yaw change: about +4.4 deg.
- Script summary suggested distance scale: 0.8333.
- Interpretation: the new WSL -> ROS -> Pi bridge -> Arduino `drive_lr` command path is working. Distance is now about 20% long, and the robot still drifts left; right-side tick travel is about 7% higher than left-side tick travel.
- Applied next action: added `distance_scale = 0.8333333333333334` to the Pi serial bridge linear target calculation and launch defaults. This scales only `/move_distance_mm`; rotation commands are unchanged.
- Next validation: rerun a 200 mm trial. After distance magnitude is back near target, tune the linear left/right balancing term for drift.

## 2026-04-21: Distance Scale Validation, 200 mm

- Log folder: `calibration_logs/20260421_013426_distance`
- Commanded distance: 200 mm.
- Measured physical distance: 175 mm.
- Physical drift: about 15 mm left.
- Encoder ticks: left 0 -> 152, right 0 -> 159.
- Tick deltas: left +152, right +159.
- Odom distance field is not trusted for this run because the start/end `/odom` snapshots jumped by several meters, which is inconsistent with the encoder and tape-measure data.
- Script summary suggested scale from this run alone: 1.1429 relative to the active `distance_scale = 0.8333`.
- Applied next action: adjusted `distance_scale` to `0.9523809523809523`.
- Interpretation: the previous 0.8333 scale over-corrected. The new value uses the latest tape/tick result to move the 200 mm command back toward target while staying below the original unscaled command.
- Next validation: rerun a 200 mm trial after restarting Pi bringup.

## 2026-04-21: Distance Scale Confirmed, 200 mm

- Log folder: `calibration_logs/20260421_013922_distance`
- Active `distance_scale`: `0.9523809523809523`.
- Commanded distance: 200 mm.
- Measured physical distance: 200 mm.
- Physical drift: about 20 mm left.
- Encoder ticks: left 0 -> 158, right 0 -> 181.
- Tick deltas: left +158, right +181.
- Odom distance: about 197.1 mm.
- Odom yaw change: about +7.3 deg.
- Interpretation: linear distance magnitude is now calibrated well enough for 200 mm trials. The remaining issue is straightness: the right side is still accumulating more ticks than the left, matching the observed left drift.
- Applied next action: increased `linear_balance_kp` from `0.4` to `0.8` while keeping `distance_scale` unchanged.
- Next validation: rerun the same 200 mm distance trial and compare drift plus left/right tick delta.

## 2026-04-21: Encoder Label Swap Found

- Log folder: `calibration_logs/20260421_014654_distance`
- Active `linear_balance_kp`: `0.8`.
- Commanded distance: 200 mm.
- Operator note: "22cm fwd, 40mm left drift". The numeric `measured_distance_mm` was entered as `22.0`, so the note is trusted as 220 mm.
- Logged ticks: `left_ticks` +167, `right_ticks` +198.
- Firmware inspection showed Arduino serial output is `E:<ticksFR>,<ticksRL>`.
- `ticksFR` is front-right and `ticksRL` is rear-left, but `serial_bridge_py` was parsing the first value as left and the second value as right.
- Interpretation: `/left_ticks` and `/right_ticks` were swapped. The balancing loop was using the wrong encoder labels, so increasing `linear_balance_kp` could make drift worse.
- Applied next action: parse `E:` as `right,left`, so ROS `/left_ticks` now comes from `ticksRL` and `/right_ticks` comes from `ticksFR`. Reset `linear_balance_kp` to `0.4` for the first corrected-label validation.
- Next validation: restart Pi bringup and rerun the same 200 mm distance trial.

## 2026-04-21: Corrected Encoder Labels Validation, 200 mm

- Log folder: `calibration_logs/20260421_015403_distance`
- Active `distance_scale`: `0.9523809523809523`.
- Active `linear_balance_kp`: `0.4`.
- Commanded distance: 200 mm.
- Measured physical distance: 200 mm.
- Physical drift: about 30 mm left.
- Encoder ticks after label correction: left +178, right +172.
- Odom start/end snapshots are not trusted for distance on this run because the pose jumped several meters, but tape distance and encoder deltas are usable.
- Interpretation: distance magnitude remains correct and encoder labels now look plausible, but physical drift is still left even when encoder ticks are close. Tick balancing alone is not enough for straightness on this floor/drive geometry.
- Applied next action: added `linear_steer_bias = 6.0`. Positive bias raises left-side PWM and lowers right-side PWM during linear moves, nudging the robot right to counter left drift. Kept `linear_balance_kp = 0.4`.
- Next validation: restart Pi bringup and rerun the 200 mm trial. Watch whether drift drops below 30 mm while distance stays near 200 mm.

## 2026-04-21: Positive Steering Bias Validation, 200 mm

- Log folder: `calibration_logs/20260421_020201_distance`
- Active `distance_scale`: `0.9523809523809523`.
- Active `linear_balance_kp`: `0.4`.
- Active `linear_steer_bias`: `6.0`.
- Commanded distance: 200 mm.
- Measured physical distance: 200 mm.
- Physical drift: about 30 mm left, noted as same as the previous trial.
- Encoder ticks: left +184, right +163.
- Odom distance: about 201.8 mm.
- Odom yaw change: about -6.7 deg.
- Interpretation: distance remains excellent, but positive steering bias did not reduce observed left drift and increased left/right tick asymmetry. The fixed bias direction is not helping this chassis/floor setup.
- Applied next action: reversed fixed steering bias to `linear_steer_bias = -6.0` for one controlled validation while keeping `distance_scale = 0.9523809523809523` and `linear_balance_kp = 0.4`.
- Next validation: rerun the 200 mm trial and compare drift direction/magnitude.

## 2026-04-21: Negative Steering Bias Validation, 200 mm

- Log folder: `calibration_logs/20260421_020717_distance`
- Active `distance_scale`: `0.9523809523809523`.
- Active `linear_balance_kp`: `0.4`.
- Active `linear_steer_bias`: `-6.0`.
- Commanded distance: 200 mm.
- Measured physical distance: 200 mm.
- Physical drift: about 1 mm right.
- Odom distance: about 204.2 mm.
- Odom yaw change: about +2.2 deg.
- Tick caveat: right tick snapshot was blank in this run, so do not use this run for left/right tick balance analysis.
- Interpretation: negative steering bias is the best 200 mm result so far. Distance remained exact and physical lateral drift dropped from about 30 mm left to about 1 mm right.
- Next action: stop tuning on 200 mm and validate at a longer distance, preferably 400 mm. If 400 mm lands near target with modest drift, keep the current values and move on to rotation/square validation.

## 2026-04-21: Longer Distance Validation, 400 mm

- Log folder: `calibration_logs/20260421_021458_distance`
- Active `distance_scale`: `0.9523809523809523`.
- Active `linear_balance_kp`: `0.4`.
- Active `linear_steer_bias`: `-6.0`.
- Commanded distance: 400 mm.
- Measured physical distance: 375 mm.
- Physical drift: about 5 mm right.
- Encoder ticks: left 172 -> 503, right 179 -> 520.
- Tick deltas: left +331, right +341.
- Odom distance: about 390.9 mm.
- Odom yaw change: about +3.2 deg.
- Interpretation: straightness is good at 400 mm, but physical distance is about 6.25% short. Odom is closer to commanded distance than the tape measurement, but tape is the calibration reference.
- Applied next action: nudged `distance_scale` to `1.0158730158730158` (`0.95238 * 400 / 375`) while keeping `linear_steer_bias = -6.0` and `linear_balance_kp = 0.4`.
- Next validation: rerun a 400 mm trial. If it lands near 400 mm with low drift, keep these values.

## 2026-04-21: Forward Calibration Accepted, 400 mm

- Log folder: `calibration_logs/20260421_022036_distance`
- Active `distance_scale`: `1.0158730158730158`.
- Active `linear_balance_kp`: `0.4`.
- Active `linear_steer_bias`: `-6.0`.
- Commanded distance: 400 mm.
- Measured physical distance: 403 mm.
- Physical drift: about 15 mm right.
- Encoder ticks: left 0 -> 360, right 0 -> 371.
- Tick deltas: left +360, right +371.
- Odom distance: about 425.2 mm.
- Odom yaw change: about +3.5 deg.
- Interpretation: physical distance error is only +0.75%, and lateral drift is acceptable for a single 400 mm open-loop segment. Stop tuning straight-line distance on single segments for now.
- Current accepted forward values:
  - `distance_scale = 1.0158730158730158`
  - `linear_balance_kp = 0.4`
  - `linear_steer_bias = -6.0`
- Next action: run square/path validation to see how the calibrated straight segments and existing rotation calibration compose over multiple moves.

## 2026-04-21: Calibrated Square Path Validation

- Log folder: `calibration_logs/20260421_022534_square`
- Sequence: four 400 mm forward moves and four +90 deg turns.
- Pause after each segment: 6.0 s.
- Active forward calibration:
  - `distance_scale = 1.0158730158730158`
  - `linear_balance_kp = 0.4`
  - `linear_steer_bias = -6.0`
- Physical final pose note: about 10 cm forward, 8 cm right, and 50 deg from start.
- Numeric entered fields:
  - `final_x_error = 100 mm`
  - `final_y_error = 80 mm`
  - `final_heading_error = 30 deg`
- The notes field says 50 deg heading error, so treat the exact heading as uncertain but still significantly nonzero.
- Odom start/end delta: x about +391 mm, y about -214 mm, yaw about +167.6 deg.
- IMU yaw delta: about +26.2 deg.
- Interpretation: square/path error improved dramatically compared with the earlier automatic square test, but the final heading is still off enough that rotation composition is now the main issue. Straight segments are acceptable; repeated turns are accumulating heading error.
- Next action: rerun rotation validation with the current code, especially +90 deg repeated 3 times and +360 deg once. Use physical measured angle as the source of truth, then retune `rotation_scale` if needed.

## 2026-04-21: Rotation Legacy Path Failure

- Log folder: `calibration_logs/20260421_023442_rotation`
- Commanded rotation: +90 deg.
- Operator note: robot spun about a full 360 deg plus roughly 39 deg more, all to the left.
- Numeric measured angle was entered as 30 deg, but the freeform note is the trusted physical observation.
- Bridge log showed `target_ticks=213` and completion around `traveled=224`, so the encoder stop condition fired even though physical rotation was far too large.
- Interpretation: legacy rotation commands (`e/q`) are no longer a trustworthy rotation path. Do not retune from the numeric CSV summary for this run.
- Applied next action: moved rotation control to `drive_lr` just like linear control, added `rotation_speed = 60`, and reset `rotation_scale = 1.0` for fresh calibration.
- New rotation command convention:
  - positive `/rotate_angle_deg` sends `drive_lr:-rotation_speed,+rotation_speed`
  - negative `/rotate_angle_deg` sends `drive_lr:+rotation_speed,-rotation_speed`
- Next validation: restart Pi bringup and run one isolated +90 deg trial.

## 2026-04-21: Rotation Drive-LR Backlog Fix

- Log folder: `calibration_logs/20260421_024230_rotation`
- Active `rotation_scale`: `1.0`.
- Active `rotation_speed`: `60`.
- Commanded rotation: +90 deg.
- Operator note: robot rotated about 360 deg plus 45 deg to the right.
- Bridge log showed `target_ticks=142` and `Motion complete traveled=150`, but Arduino continued acknowledging `drive_lr:-60,60` commands afterward.
- Interpretation: the bridge was sending rotation `drive_lr` commands every control tick, creating a serial backlog. The stop command could arrive late, allowing large physical overspin. The turn direction is also not yet calibrated.
- Applied next action:
  - Added `rotation_command_interval_s = 0.75` so rotation drive commands refresh slowly instead of flooding serial.
  - Added explicit `drive_lr:0,0` before `x` on motion complete, timeout, and cancel.
  - Kept `rotation_scale = 1.0` and `rotation_speed = 60` for the next isolated test.
- Next validation: restart Pi bringup and run one isolated +90 deg trial. If direction is still wrong but overspin is controlled, flip rotation sign next.

## 2026-04-21: Throttled Rotation Validation, +90 deg

- Log folder: `calibration_logs/20260421_104308_rotation`
- Active `rotation_scale`: `1.0`.
- Active `rotation_speed`: `60`.
- Active `rotation_command_interval_s`: `0.75`.
- Commanded rotation: +90 deg.
- Measured physical rotation: about 45 deg.
- Operator note: "45 deg to the right, no weirdness".
- Direction was marked correct by the operator.
- Encoder ticks: left 0 -> -134, right 0 -> 171.
- Odom yaw delta: about +96.8 deg.
- IMU yaw delta: about +7.4 deg.
- Interpretation: the serial backlog/runaway rotation problem is fixed. Physical rotation now under-rotates by about 50%, while odom/IMU remain inconsistent with the tape/visual measurement.
- Applied next action: set `rotation_scale = 2.0`, keeping `rotation_speed = 60` and `rotation_command_interval_s = 0.75`.
- Next validation: restart Pi bringup and rerun one isolated +90 deg trial.

## 2026-04-21: Rotation Scale 2.0 Validation, +90 deg

- Log folder: `calibration_logs/20260421_104856_rotation`
- Active `rotation_scale`: `2.0`.
- Active `rotation_speed`: `60`.
- Active `rotation_command_interval_s`: `0.75`.
- Commanded rotation: +90 deg.
- Measured physical rotation: about 225 deg to the right.
- Operator note: "overshot hella".
- Encoder ticks: left 0 -> -418, right 0 -> 512.
- Pi bridge log showed `target_ticks=283` and `Motion complete traveled=287`, but one more `drive_lr:-60,60` command was acknowledged after completion before `drive_lr:0,0`.
- Interpretation: scale 2.0 is too high, and rotation still has enough stop latency/coast that high speed magnifies overshoot. The direction is usable, but calibration should continue conservatively.
- Applied next action: set `rotation_scale = 1.2` and lower `rotation_speed = 40`, keeping `rotation_command_interval_s = 0.75`.
- Next validation: restart Pi bringup and rerun one isolated +90 deg trial.

## 2026-04-21: Rotation Motor Deadband Check

- Low-level direct serial check showed `drive_lr:0,40` reached the Arduino but barely moved the right encoder, only a few ticks.
- Operator checked higher PWM values and confirmed the wheels move at higher PWM.
- Interpretation: `rotation_speed = 40` is too close to motor deadband for trustworthy rotation calibration.
- Applied next action: set `rotation_speed = 80` and reset `rotation_scale = 1.0` for the next clean isolated +90 deg calibration trial.
- Keep `rotation_command_interval_s = 0.75` and the explicit `drive_lr:0,0` stop behavior.

## 2026-04-21: Rotation Speed 80 Validation, +90 deg

- Log folder: `calibration_logs/20260421_110621_rotation`
- Active `rotation_scale`: `1.0`.
- Active `rotation_speed`: `80`.
- Active `rotation_command_interval_s`: `0.75`.
- Commanded rotation: +90 deg.
- Measured physical rotation: about 135 deg.
- Operator note: all tires moved fine.
- Direction was marked correct.
- Encoder ticks: left 419 -> 99, right 563 -> 925.
- Tick deltas: left -320, right +362.
- Odom yaw delta: about -143.5 deg.
- IMU yaw delta: about +6.8 deg.
- Interpretation: PWM 80 clears the motor deadband, so this is usable rotation-scale data. Physical rotation overshot by about 50%.
- Applied next action: set `rotation_scale = 0.6666666666666666`, keeping `rotation_speed = 80`.
- Next validation: restart Pi bringup and rerun one isolated +90 deg trial.
