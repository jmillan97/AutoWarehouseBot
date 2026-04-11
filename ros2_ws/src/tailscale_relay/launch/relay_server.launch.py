"""Standalone launch for the relay server (Pi-side). For testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        Node(
            package='tailscale_relay',
            executable='relay_server',
            name='relay_server',
            output='screen',
            parameters=[{'port': LaunchConfiguration('port')}],
        ),
    ])
