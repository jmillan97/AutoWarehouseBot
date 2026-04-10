"""
robot_bringup.launch.py
=======================
Physical hardware bringup launch file — runs on the Raspberry Pi (Pi 3, 1GB RAM).
Kept intentionally minimal: only nodes that need direct hardware access run here.

Starts:
  1. RPLidar A1 driver    → publishes /scan
  2. serial_bridge node   → bridges Arduino serial ↔ /left_ticks, /right_ticks, /cmd_vel
  3. wheel_odometry node  → computes /odom from encoder ticks

Does NOT start on Pi (runs on laptop Docker — see navigation/launch/hardware.launch.py):
  - robot_state_publisher  (URDF + TF)
  - EKF node               (fuses /odom + /imu/data)
  - Nav2 (AMCL, planner, MPPI controller)
  - SLAM / Cartographer

Usage (on Pi):
  ros2 launch embedded robot_bringup.launch.py

  Optional args:
    lidar_port:=/dev/lidar      (default — udev symlink)
    serial_port:=/dev/arduino   (default — udev symlink)
    serial_baud:=115200          (default)
    use_sim_time:=false          (default — always false on real hardware)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch arguments ----
    # Separate ports for each device — udev symlinks make these permanent.
    # See /etc/udev/rules.d/99-robot-devices.rules on the Pi.
    lidar_port   = LaunchConfiguration('lidar_port')
    serial_port  = LaunchConfiguration('serial_port')
    serial_baud  = LaunchConfiguration('serial_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('lidar_port',   default_value='/dev/lidar'),
        DeclareLaunchArgument('serial_port',  default_value='/dev/arduino'),
        DeclareLaunchArgument('serial_baud',  default_value='115200'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # ---- 1. RPLidar A1 driver ----
    # Uses rplidar_ros (system-installed on Pi via apt, confirmed working).
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port':      lidar_port,
            'serial_baudrate':  115200,
            'frame_id':         'laser',
            'inverted':         False,
            'angle_compensate': True,
            'use_sim_time':     use_sim_time,
        }]
    )

    # ---- 2. Serial bridge (Arduino ↔ ROS2) ----
    serial_bridge = Node(
        package='embedded',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port':  serial_port,
            'serial_baud':  serial_baud,
            'use_sim_time': use_sim_time,
        }]
    )

    # ---- 3. Wheel odometry ----
    wheel_odometry = Node(
        package='embedded',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'wheel_radius':     0.04,
            'wheel_separation': 0.21,
            'encoder_cpr':      2.0,
            'gear_ratio':       30.0,   # [TUNE] measure on real hardware
            'use_sim_time':     use_sim_time,
        }]
    )

    return LaunchDescription(
        args + [
            lidar,
            serial_bridge,
            wheel_odometry,
        ]
    )
