"""Standalone launch for the relay client (laptop-side). For testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pi_address', default_value='100.91.37.52'),
        DeclareLaunchArgument('port', default_value='8765'),
        Node(
            package='tailscale_relay',
            executable='relay_client',
            name='relay_client',
            output='screen',
            parameters=[{
                'pi_address': LaunchConfiguration('pi_address'),
                'port': LaunchConfiguration('port'),
            }],
        ),
    ])
