"""
hardware.launch.py
==================
Laptop-side launch file for real hardware operation.
Runs in Docker on the offboard compute machine.

Starts:
  1. robot_state_publisher  — URDF + TF (odom → base_link)
  2. EKF node               — fuses /odom from Pi + /imu/data → /odom_filtered
  3. Nav2                   — AMCL, planner, MPPI controller

Counterpart: ros2 launch embedded robot_bringup.launch.py  (runs on Pi)

Usage (in Docker):
  ros2 launch navigation hardware.launch.py

  Optional args:
    map:=<path>          (default: blank_map.yaml)
    use_sim_time:=false  (always false for real hardware)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    wb_navigation_dir  = get_package_share_directory('navigation')
    nav2_bringup_dir   = get_package_share_directory('nav2_bringup')
    wb_description_dir = get_package_share_directory('description')
    wb_embedded_dir    = get_package_share_directory('embedded')

    map_file    = os.path.join(wb_navigation_dir, 'maps', 'blank_map.yaml')
    nav2_params = os.path.join(wb_navigation_dir, 'config', 'nav2_params.yaml')
    urdf_file   = os.path.join(wb_description_dir, 'urdf', 'warehouse_bot.urdf.xacro')
    ekf_config  = os.path.join(wb_embedded_dir,    'config', 'ekf_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_file),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ---- 1. Robot state publisher ----
    # Publishes URDF and odom→base_link TF. Runs on laptop so Nav2 can use it
    # without adding another process to the memory-constrained Pi 3.
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

    # ---- 2. EKF node ----
    # Subscribes to /odom (from Pi's wheel_odometry) + /imu/data.
    # Publishes /odom_filtered consumed by Nav2 AMCL.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odom_filtered'),
        ]
    )

    # ---- 3. Nav2 ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items()
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        ekf,
        nav2,
    ])
