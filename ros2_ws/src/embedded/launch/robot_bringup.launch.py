"""
robot_bringup.launch.py
=======================
Physical hardware bringup launch file — runs on the Raspberry Pi.
This is the hardware equivalent of gazebo.launch.py.

Starts:
  1. RPLidar A1 driver         → publishes /scan          (/dev/ttyUSB1)
  2. serial_bridge node        → bridges Arduino serial    (/dev/ttyUSB0)
  3. wheel_odometry node       → computes /odom from encoder ticks
  4. robot_state_publisher     → URDF + static TF transforms
  5. EKF node                  → fuses /odom + /imu/data
  6. usb_cam node              → publishes /camera/image_raw

Does NOT start on Pi (runs on offboard PC):
  - Nav2 (AMCL, planner, MPPI controller)
  - SLAM / Cartographer

Usage (on Pi):
  ros2 launch embedded robot_bringup.launch.py

  Optional args:
    serial_port:=/dev/ttyUSB0   (Arduino, default)
    serial_baud:=9600            (default)
    use_sim_time:=false          (always false on real hardware)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit



def generate_launch_description():

    # ---- Package directories ----
    embedded_dir    = get_package_share_directory('embedded')
    description_dir = get_package_share_directory('description')

    # ---- File paths ----
    urdf_file  = os.path.join(description_dir, 'urdf', 'warehouse_bot.urdf.xacro')
    ekf_config = os.path.join(embedded_dir, 'config', 'ekf_params.yaml')

    # ---- Launch arguments ----
    serial_port  = LaunchConfiguration('serial_port')
    serial_baud  = LaunchConfiguration('serial_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('serial_port',  default_value='/dev/arduino'),
        DeclareLaunchArgument('serial_baud',  default_value='115200'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # ---- Robot description (URDF) ----
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ---- 1. RPLidar A1 driver ----
    # LiDAR confirmed on /dev/ttyUSB1
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/lidar',
            'serial_baudrate':  115200,
            'frame_id':         'base_laser',
            'inverted':         False,
            'angle_compensate': True,
        }]
    )
    lidar_respawn = RegisterEventHandler(
        OnProcessExit(
        target_action=lidar,
        on_exit=[
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='rplidar_ros',
                        executable='rplidar_composition',
                        name='rplidar_node',
                        output='screen',
                        parameters=[{
                            'serial_port':      '/dev/lidar',
                            'serial_baudrate':  115200,
                            'frame_id':         'base_laser',
                            'inverted':         False,
                            'angle_compensate': True,
                            }]
                        )
                    ]  
                )
            ]
        )
    )

    # ---- 2. Serial bridge (Arduino ↔ ROS2) ----
    # Arduino confirmed on /dev/ttyUSB0
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
            'gear_ratio':       30.0,   # [TUNE] measure on real hardware
            'use_sim_time':     use_sim_time,
        }]
    )

    # ---- 4. Robot state publisher ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time':      use_sim_time,
        }]
    )

    # ---- 5. EKF node ----
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odometry/filtered', '/odom_filtered'),
        ]
    )

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

    # ---- 6. USB Camera ----
    # pixel_format=mjpeg: camera does JPEG compression in hardware, cutting USB
    # bandwidth ~4x vs raw YUYV — prevents the V4L2 select() timeout crash under
    # Pi CPU load. Camera confirmed to support Motion-JPEG at 640x480@30fps.
    camera_params = {
        'video_device':    '/dev/video0',
        'image_width':     640,
        'image_height':    480,
        'framerate':       30.0,
        'pixel_format':    'mjpeg',
        'camera_frame_id': 'camera_optical_link',
    }
    camera_remaps = [
        ('image_raw',   '/camera/image_raw'),
        ('camera_info', '/camera/camera_info'),
    ]

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[camera_params],
        remappings=camera_remaps,
    )

    camera_respawn = RegisterEventHandler(
        OnProcessExit(
            target_action=camera,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package='usb_cam',
                            executable='usb_cam_node_exe',
                            name='usb_cam',
                            output='screen',
                            parameters=[camera_params],
                            remappings=camera_remaps,
                        )
                    ]
                )
            ]
        )
    )

    # 0.5s delay to prevent simultaneous USB init with LiDAR on startup
    delayed_camera = TimerAction(
        period=0.5,
        actions=[camera]
    )

    return LaunchDescription(
        args + [
            robot_state_publisher,
            TimerAction(period=2.0, actions=[lidar]),
            lidar_respawn,
            serial_bridge,
            wheel_odometry,
            ekf,
            delayed_camera,
            camera_respawn,
            imu
        ]
    )
