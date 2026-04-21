#!/usr/bin/env python3
"""
imu_node.py
===========
ROS2 node that reads from the Adafruit 10-DOF IMU breakout
(L3GD20H + LSM303DLHC) via RTIMULib and publishes to /imu/data.

Hardware:
  - L3GD20H  gyroscope
  - LSM303   accelerometer + magnetometer
  Connected via I2C to Raspberry Pi

Publishes:
  /imu/data  (sensor_msgs/Imu)       — fused orientation + angular vel + linear accel
  /imu/mag   (sensor_msgs/MagneticField) — raw magnetometer data

Parameters:
  imu_frame_id   (string)  default: imu_link
  publish_rate   (int)     default: 50
  ini_file       (string)  default: /home/ece_441/RTIMULib.ini
"""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

try:
    import RTIMU
except ImportError:
    print("ERROR: RTIMU not found. Install with: sudo apt install python3-rtimulib")
    sys.exit(1)


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ---- Parameters ----
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50)
        self.declare_parameter('ini_file', '/home/ece_441/RTIMULib.ini')
        self.declare_parameter('orientation_covariance_diagonal', [0.05, 0.05, 0.10])
        self.declare_parameter('angular_velocity_covariance_diagonal', [0.001, 0.001, 0.001])
        self.declare_parameter('linear_acceleration_covariance_diagonal', [0.01, 0.01, 0.01])
        self.declare_parameter('magnetic_field_covariance_diagonal', [0.001, 0.001, 0.001])

        self.frame_id = self.get_parameter('imu_frame_id').value
        self.rate     = self.get_parameter('publish_rate').value
        ini_file      = self.get_parameter('ini_file').value
        self.orientation_covariance = self._diag_covariance(
            self.get_parameter('orientation_covariance_diagonal').value,
            fallback=[0.05, 0.05, 0.10],
        )
        self.angular_velocity_covariance = self._diag_covariance(
            self.get_parameter('angular_velocity_covariance_diagonal').value,
            fallback=[0.001, 0.001, 0.001],
        )
        self.linear_acceleration_covariance = self._diag_covariance(
            self.get_parameter('linear_acceleration_covariance_diagonal').value,
            fallback=[0.01, 0.01, 0.01],
        )
        self.magnetic_field_covariance = self._diag_covariance(
            self.get_parameter('magnetic_field_covariance_diagonal').value,
            fallback=[0.001, 0.001, 0.001],
        )

        # ---- RTIMULib setup ----
        ini_dir  = os.path.dirname(ini_file)
        ini_name = os.path.splitext(os.path.basename(ini_file))[0]

        self.settings = RTIMU.Settings(os.path.join(ini_dir, ini_name))
        self.imu      = RTIMU.RTIMU(self.settings)

        if not self.imu.IMUInit():
            self.get_logger().fatal("IMU init failed — check I2C wiring and address")
            raise RuntimeError("IMU init failed")

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        self.get_logger().info(f"IMU initialized: {self.imu.IMUName()}")
        self.get_logger().info(f"Publishing at {self.rate}Hz on /imu/data")

        # ---- Publishers ----
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # ---- Timer ----
        poll_interval = self.imu.IMUGetPollInterval() / 1000.0
        timer_period  = max(1.0 / self.rate, poll_interval)
        self.create_timer(timer_period, self.publish_imu)

    def _diag_covariance(self, values, fallback):
        try:
            diag = [float(v) for v in values]
        except TypeError:
            diag = list(fallback)
        if len(diag) != 3:
            self.get_logger().warn(f'Expected 3 covariance diagonal values, got {diag}; using {fallback}')
            diag = list(fallback)
        return [
            diag[0], 0.0,     0.0,
            0.0,     diag[1], 0.0,
            0.0,     0.0,     diag[2],
        ]

    def publish_imu(self):
        if not self.imu.IMURead():
            return

        data = self.imu.getIMUData()
        now  = self.get_clock().now().to_msg()

        # ---- IMU message ----
        imu_msg = Imu()
        imu_msg.header.stamp    = now
        imu_msg.header.frame_id = self.frame_id

        # Orientation (fused quaternion from RTIMULib)
        if data.get('fusionQPoseValid', False) and 'fusionQPose' in data:
            q = data['fusionQPose']
            imu_msg.orientation.x = q.x()
            imu_msg.orientation.y = q.y()
            imu_msg.orientation.z = q.z()
            imu_msg.orientation.w = q.scalar()
            imu_msg.orientation_covariance = self.orientation_covariance
        else:
            # Orientation not valid yet
            imu_msg.orientation_covariance[0] = -1.0

        # Angular velocity (gyroscope) — rad/s
        if data.get('gyroValid', False):
            gyro = data['gyro']
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        else:
            imu_msg.angular_velocity_covariance[0] = -1.0

        # Linear acceleration (accelerometer) — m/s²
        if data.get('accelValid', False):
            accel = data['accel']
            imu_msg.linear_acceleration.x = accel[0] * 9.81
            imu_msg.linear_acceleration.y = accel[1] * 9.81
            imu_msg.linear_acceleration.z = accel[2] * 9.81
            imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        else:
            imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        # ---- Magnetometer message ----
        if data.get('compassValid', False):
            mag_msg = MagneticField()
            mag_msg.header.stamp    = now
            mag_msg.header.frame_id = self.frame_id
            compass = data['compass']
            # Convert uT to Tesla
            mag_msg.magnetic_field.x = compass[0] * 1e-6
            mag_msg.magnetic_field.y = compass[1] * 1e-6
            mag_msg.magnetic_field.z = compass[2] * 1e-6
            mag_msg.magnetic_field_covariance = self.magnetic_field_covariance
            self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
