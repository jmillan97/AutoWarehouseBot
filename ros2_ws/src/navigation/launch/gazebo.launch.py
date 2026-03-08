"""
gazebo.launch.py
================
Launches the full simulation stack:
  1. Gazebo Ionic with warehouse_bot.sdf
  2. ros_gz_bridge — maps Gazebo topics to ROS2 topics
  3. robot_state_publisher — URDF + TF static transforms
  4. joint_state_publisher — wheel joint states
  5. Nav2 full stack (map_server, AMCL, planner, MPPI controller)

This replaces all fake sensor nodes — Gazebo provides real simulated
/scan, /odom, and /tf data.

Usage:
  ros2 launch wb_navigation gazebo.launch.py
  ros2 launch wb_navigation gazebo.launch.py use_rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():

    # ---- Package directories ----
    wb_navigation_dir  = get_package_share_directory('navigation')
    wb_description_dir = get_package_share_directory('description')
    nav2_bringup_dir   = get_package_share_directory('nav2_bringup')
    pkg_ros_gz_sim     = get_package_share_directory('ros_gz_sim')

    # ---- File paths ----
    sdf_file    = os.path.join(wb_navigation_dir, 'worlds', 'warehouse_bot.sdf')
    bridge_yaml = os.path.join(wb_navigation_dir, 'config', 'bridge.yaml')
    nav2_params = os.path.join(wb_navigation_dir, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(wb_navigation_dir, 'maps', 'blank_map.yaml')
    urdf_file   = os.path.join(wb_description_dir, 'urdf', 'warehouse_bot.urdf.xacro')

    # ---- Launch arguments ----
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_rviz',     default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    # ---- Robot description ----
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ---- 1. Gazebo Ionic ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r = run immediately, -v 4 = verbose for debugging
            'gz_args': f'-r -v 4 {sdf_file}',
        }.items(),
    )

    # ---- 2. ROS-Gazebo Bridge ----
    bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_yaml,
    )

    # ---- 3. Robot state publisher ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # ---- 4. Joint state publisher ----
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---- 5. Nav2 bringup ----
    # Delayed to let Gazebo and bridge fully initialize first
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map':          map_file,
                    'use_sim_time': use_sim_time,
                    'params_file':  nav2_params,
                    'autostart':    'true',
                }.items()
            )
        ]
    )

    # ---- 6. RViz (optional) ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args + [
            gazebo,
            bridge,
            robot_state_publisher,
            joint_state_publisher,
            nav2,
            rviz,
        ]
    )
