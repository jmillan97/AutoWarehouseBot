"""
zenoh_relay.launch.py
=====================
Launches the Zenoh relay node that bridges ROS2 topics between the laptop
(WSL) and the Raspberry Pi through a Zenoh router on the Windows host.

This file is meant to be INCLUDED inside other launch files, not run
standalone — see the "Including this launch file" section below.

  On Pi   — included at the bottom of:  ros2 launch embedded robot_bringup.launch.py
  On WSL  — included at the bottom of:  ros2 launch navigation hardware.launch.py

Both sides connect outward to the same Windows zenohd router, so no
inbound firewall rules are needed on WSL (which is behind Windows NAT).

Including this launch file
--------------------------
Add these lines to robot_bringup.launch.py or hardware.launch.py:

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    # In generate_launch_description(), alongside your other args:
    DeclareLaunchArgument('router_ip', default_value='192.168.1.100'),

    # In the return LaunchDescription list:
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('embedded'),
                         'launch', 'zenoh_relay.launch.py')
        ),
        launch_arguments={
            'role':      'pi',              # or 'laptop'
            'router_ip': LaunchConfiguration('router_ip'),
        }.items(),
    ),

Standalone usage (debugging only)
----------------------------------
  # On Pi:
  ros2 launch embedded zenoh_relay.launch.py role:=pi router_ip:=192.168.1.100

  # On WSL:
  ros2 launch embedded zenoh_relay.launch.py role:=laptop router_ip:=192.168.1.100

Arguments
---------
  role        : "laptop" or "pi"  (required — no default)
  router_ip   : IP of the Windows host running zenohd.exe
                Find it: ipconfig → look for your WiFi adapter's IPv4 address
                Default : 192.168.1.100  (update to match your network)
  router_port : Zenoh router TCP port (default 7447, rarely needs changing)

Prerequisites
-------------
  Windows host : zenohd.exe running (download from github.com/eclipse-zenoh/zenoh/releases)
  WSL + Pi     : pip install eclipse-zenoh

Dead network detection
----------------------
  - Zenoh liveliness tokens: if the peer relay exits (crash or clean shutdown),
    a DELETE event fires on this side within ~1 s. You will see a WARN in the log.
  - Watchdog thread: checks the Zenoh session every 5 s and logs an ERROR if
    the router (zenohd.exe) is unreachable (e.g. Windows machine rebooted).
  - No heartbeat topic is published; all health monitoring is internal.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    role_arg = DeclareLaunchArgument(
        "role",
        description='Which machine this relay runs on: "laptop" or "pi".',
    )

    router_ip_arg = DeclareLaunchArgument(
        "router_ip",
        default_value="192.168.1.100",
        description=(
            "IPv4 address of the Windows host running zenohd.exe. "
            "Run `ipconfig` on Windows and look for your WiFi adapter."
        ),
    )

    router_port_arg = DeclareLaunchArgument(
        "router_port",
        default_value="7447",
        description="TCP port zenohd listens on (default 7447).",
    )

    # ── Relay node ─────────────────────────────────────────────────────────────
    relay_node = Node(
        package="embedded",
        executable="zenoh_relay",
        name="zenoh_relay",
        output="screen",
        parameters=[{
            "role":        LaunchConfiguration("role"),
            "router_ip":   LaunchConfiguration("router_ip"),
            "router_port": LaunchConfiguration("router_port"),
        }],
    )

    return LaunchDescription([
        role_arg,
        router_ip_arg,
        router_port_arg,
        relay_node,
    ])
