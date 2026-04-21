"""
slam_mapping.launch.py
======================
AutoWarehouseBot — ros2 branch
Laptop / WSL2 side — run AFTER robot_bringup.launch.py is running on the Pi

What this does:
  1. Starts SLAM Toolbox in online_async mode
     → subscribes to /scan (from Pi's RPLidar)
     → publishes /map and the map→odom transform
  2. Starts RViz with a mapping-focused config

Workflow:
  Step 1 — On Pi:
    ros2 launch wb_embedded robot_bringup.launch.py

  Step 2 — On laptop (this file):
    ros2 launch wb_nav slam_mapping.launch.py

  Step 3 — Drive the robot manually:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

  Step 4 — Save the map when done:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map
    (saves warehouse_map.pgm and warehouse_map.yaml)

  Step 5 — Copy map to Pi:
    scp ~/maps/warehouse_map.* ece_441@<PI_IP>:~/maps/

Notes:
  - SLAM Toolbox runs on the laptop, not the Pi — Pi 3B+ is too slow for it
  - use_sim_time is false — this is real hardware
  - The map frame is 'map', odom frame is 'odom', base frame is 'base_link'
  - If /scan isn't arriving, check: ros2 topic hz /scan
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package dir ──────────────────────────────────────────────
    wb_nav_dir = get_package_share_directory('wb_nav')

    # ── Config files ─────────────────────────────────────────────
    slam_params_file = os.path.join(wb_nav_dir, 'config', 'slam_params.yaml')
    rviz_config_file = os.path.join(wb_nav_dir, 'rviz', 'mapping.rviz')

    # ── Launch args ──────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (always false on real hardware)'
        ),
    ]

    # ── SLAM Toolbox — online async ───────────────────────────────
    # online_async: builds map incrementally as scans arrive.
    # Runs entirely on laptop — Pi does not need slam_toolbox installed.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # /scan comes from the Pi's RPLidar driver over the network
            ('/scan', '/scan'),
        ]
    )

    # ── RViz ─────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(args + [slam_toolbox, rviz])