"""
robot_bringup.launch.py
=======================
Physical hardware bringup launch file — runs on the Raspberry Pi.
This is the hardware equivalent of gazebo.launch.py.

Starts:
  1. serial_bridge node        → bridges Arduino serial    (/dev/ttyUSB0)
  2. wheel_odometry node       → computes /odom from encoder ticks
  3. robot_state_publisher     → URDF + static TF transforms
  4. IMU node                  → publishes /imu/data
  5. usb_cam node              → publishes /camera/image_raw

Does NOT start on Pi (runs on offboard PC):
  - EKF / robot_localization
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
from launch.conditions import IfCondition
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
    # ---- Launch arguments ----
    serial_port  = LaunchConfiguration('serial_port')
    serial_baud  = LaunchConfiguration('serial_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_lidar = LaunchConfiguration('enable_lidar')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_framerate = LaunchConfiguration('camera_framerate')
    camera_pixel_format = LaunchConfiguration('camera_pixel_format')

    args = [
        DeclareLaunchArgument('serial_port',  default_value='/dev/arduino'),
        DeclareLaunchArgument('serial_baud',  default_value='115200'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Set false to disable RPLidar when /dev/lidar is disconnected or unstable.',
        ),
        DeclareLaunchArgument('camera_width', default_value='320'),
        DeclareLaunchArgument('camera_height', default_value='240'),
        DeclareLaunchArgument('camera_framerate', default_value='20.0'),
        DeclareLaunchArgument('camera_pixel_format', default_value='yuyv2rgb'),
    ]

    # ---- Robot description (URDF) ----
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ---- Optional: RPLidar A1 driver ----
    # Disabled by default because LiDAR is often unplugged during camera/YOLO work.
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        condition=IfCondition(enable_lidar),
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
                        condition=IfCondition(enable_lidar),
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
    # Python bridge is used for faster iteration while movement protocol settles.
    # Arduino confirmed on /dev/ttyUSB0
    serial_bridge = Node(
        package='embedded',
        executable='serial_bridge_py',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baud': serial_baud,
            'command_speed': 90,
            'encoder_cpr': 2.0,
            'gear_ratio': 108.0,
            'distance_scale': 1.0158730158730158,
            'rotation_scale': 0.895,
            'rotation_speed': 80,
            'rotation_command_interval_s': 0.75,
            'use_drive_lr_linear': True,
            'linear_balance_kp': 0.4,
            'linear_steer_bias': -6.0,
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
            'gear_ratio':       108.0,  # Calibrated from distance trials at command_speed=90.
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

    imu = Node(
        package='embedded',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'imu_frame_id': 'imu_link',
            'publish_rate': 50,
            'ini_file': '/home/ece_441/RTIMULib.ini',
            # Conservative starting values; replace with measured calibration data.
            'orientation_covariance_diagonal': [0.05, 0.05, 0.10],
            'angular_velocity_covariance_diagonal': [0.001, 0.001, 0.001],
            'linear_acceleration_covariance_diagonal': [0.01, 0.01, 0.01],
            'magnetic_field_covariance_diagonal': [0.001, 0.001, 0.001],
        }]
    )

    # ---- 6. USB Camera ----
    # Use the simpler YUYV capture path instead of MJPEG decoding. This avoids
    # the flaky usb_cam MJPEG timeout/crash path we observed, while existing
    # image_transport plugins can still provide /camera/image_raw/compressed
    # for offboard WSL viewing.
    camera_params = {
        'video_device':    '/dev/video0',
        'image_width':     camera_width,
        'image_height':    camera_height,
        'framerate':       camera_framerate,
        'pixel_format':    camera_pixel_format,
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

    # Give USB peripherals a moment to settle before camera init.
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
            delayed_camera,
            camera_respawn,
            imu
        ]
    )
