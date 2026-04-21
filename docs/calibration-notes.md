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
