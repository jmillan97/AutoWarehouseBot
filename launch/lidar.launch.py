from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',   # persistent udev symlink
                'serial_baudrate': 115200,     # RPLidar A1/A2
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
