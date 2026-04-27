"""
nav2_bringup.launch.py
======================
AutoWarehouseBot — ros2 branch
Laptop / WSL2 side — run AFTER robot_bringup.launch.py is running on the Pi
and you have a saved map from the SLAM mapping phase.

What this starts:
  - map_server          → serves the saved .pgm/.yaml map
  - amcl                → particle filter localization using /scan + /odom
  - nav2_controller     → MPPI controller → publishes /cmd_vel
  - nav2_planner        → SMAC planner → computes global paths
  - nav2_behaviors      → recovery behaviors (spin, backup, wait)
  - nav2_bt_navigator   → behavior tree orchestrator
  - nav2_lifecycle_mgr  → manages all node lifecycles
  - RViz

Workflow:
  Step 1 — On Pi:
    ros2 launch wb_embedded robot_bringup.launch.py

  Step 2 — On laptop (this file):
    ros2 launch wb_nav nav2_bringup.launch.py map:=/home/jmill/maps/warehouse_map.yaml

  Step 3 — In RViz:
    Click "2D Pose Estimate" and click where the robot is on the map
    → AMCL will localize from that initial guess

  Step 4 — Send a goal:
    Click "Nav2 Goal" in RViz and click a destination
    → or use the summon server

Args:
  map          path to .yaml map file  (required)
  use_sim_time false (always on real hardware)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    wb_nav_dir = get_package_share_directory('navigation')

    nav2_params_file = os.path.join(wb_nav_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(wb_nav_dir, 'rviz',   'nav2.rviz')

    # ── Launch args ──────────────────────────────────────────────
    map_yaml       = LaunchConfiguration('map')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    use_rviz       = LaunchConfiguration('use_rviz')

    args = [
        DeclareLaunchArgument(
            'map',
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),
    ]

    # ── Shared params injected into every Nav2 node ───────────────
    nav2_common_params = [
        nav2_params_file,
        {'use_sim_time': use_sim_time}
    ]
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','map','odom']
    )

    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','odom','base_link']
    )

    # ── map_server ────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml}]
    )

    # ── AMCL localization ─────────────────────────────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=nav2_common_params
    )

    # ── Controller server (MPPI) ──────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=nav2_common_params,
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # ── Planner server (SMAC lattice) ────────────────────────────
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=nav2_common_params
    )

    # ── Behavior server (recoveries) ─────────────────────────────
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=nav2_common_params,
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    # ── BT Navigator ──────────────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=nav2_common_params
    )

    # ── Velocity smoother ─────────────────────────────────────────
    # Smooths MPPI output before it hits /cmd_vel
    # Important: keeps commands from jumping around between rotate/drive states
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=nav2_common_params,
        remappings=[
            ('cmd_vel',        'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed')
        ]
    )
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file,
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_in', 'cmd_vel_smoothed'),
                    ('cmd_vel_out', '/cmd_vel')]
    )

    # ── Lifecycle manager ────────────────────────────────────────
    # Brings all Nav2 nodes through configure → activate
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'autostart':     True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'velocity_smoother',
                'collision_monitor',
            ]
        }]
    )
    lifecycle_manager_bt = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_bt',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'bond_timeout': 0.0,
                    'node_names': ['bt_navigator']
                }]
            )
        ]
    )
    lifecycle_manager_delayed = TimerAction(
        period=5.0,
        actions=[lifecycle_manager]
    )
    publish_initial_pose = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{"header":{"frame_id":"map"},"pose":{"pose":{"position":{"x":0.0,"y":0.0,"z":0.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}},"covariance":[0.5,0,0,0,0,0,0,0.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.26]}}'
                ],
                output='screen'
            )
        ]
    )
   

    # ── RViz ──────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription(args + [
        map_to_odom_tf,
        odom_to_base_tf,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        velocity_smoother,
        collision_monitor,
        bt_navigator,
        lifecycle_manager_delayed,
        lifecycle_manager_bt,
        rviz,
        publish_initial_pose,
    ])