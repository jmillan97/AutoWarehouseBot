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
