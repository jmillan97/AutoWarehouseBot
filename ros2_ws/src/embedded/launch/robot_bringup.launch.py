"""
robot_bringup.launch.py
=======================
Physical hardware bringup launch file — runs on the Raspberry Pi.
This is the hardware equivalent of gazebo.launch.py.

Starts:
  1. RPLidar A1 driver         → publishes /scan
  2. serial_bridge node        → bridges Arduino serial
  3. wheel_odometry node       → computes /odom from encoder ticks
  4. IMU node                  → publishes /imu/data
  5. usb_cam node              → publishes /camera/image_raw
  6. relay_server              → WebSocket relay to laptop over Tailscale

EKF and robot_state_publisher run on the laptop (hardware.launch.py).

Does NOT start on Pi (runs on offboard PC):
  - Nav2 (AMCL, planner, MPPI controller)
  - EKF (fuses /odom + /imu/data on laptop)
  - robot_state_publisher (URDF + TF on laptop)

Usage (on Pi):
  ros2 launch embedded robot_bringup.launch.py

  Optional args:
    serial_port:=/dev/ttyUSB0   (Arduino, default)
    serial_baud:=115200          (default)
    use_sim_time:=false          (always false on real hardware)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch arguments ----
    serial_port  = LaunchConfiguration('serial_port')
    serial_baud  = LaunchConfiguration('serial_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('serial_port',  default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud',  default_value='115200'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # ---- 1. RPLidar A1 driver ----
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'serial_port':      '/dev/lidar',
            'serial_baudrate':  115200,
            'frame_id':         'base_laser',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }]
    )

    delayed_lidar = TimerAction(
        period=4.0,
        actions=[lidar]
    )

    # ---- 2. Serial bridge (Arduino <-> ROS2) ----
    serial_bridge = Node(
        package='embedded',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baud': serial_baud,
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
            'gear_ratio':       30.0,
            'use_sim_time':     use_sim_time,
        }]
    )

    # ---- 4. IMU node ----
    imu = Node(
        package='embedded',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'imu_frame_id': 'imu_link',
            'publish_rate': 50,
            'ini_file': '/home/ece_441/RTIMULib.ini',
        }]
    )

    # ---- 5. USB Camera ----
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device':    '/dev/video0',
            'image_width':     640,
            'image_height':    480,
            'framerate':       30.0,
            'camera_frame_id': 'camera_optical_link',
        }],
        remappings=[
            ('image_raw',   '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    delayed_camera = TimerAction(
        period=0.5,
        actions=[camera]
    )

    # ---- 6. Tailscale Relay server ----
    relay_server = Node(
        package='tailscale_relay',
        executable='relay_server',
        name='relay_server',
        output='screen',
        parameters=[{'port': 8765}],
    )

    return LaunchDescription(
        args + [
            delayed_lidar,
            serial_bridge,
            wheel_odometry,
            imu,
            delayed_camera,
            relay_server,
        ]
    )
