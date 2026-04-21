from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    wb_summon_dir = get_package_share_directory('summon')

    return LaunchDescription([

        DeclareLaunchArgument('initial_pose_x',     default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y',     default_value='0.0'),
        DeclareLaunchArgument('initial_pose_theta', default_value='0.0'),

        Node(
            package='summon',
            executable='summon_node',
            name='summon_node',
            output='screen',
            parameters=[{
                'initial_pose_x':          LaunchConfiguration('initial_pose_x'),
                'initial_pose_y':          LaunchConfiguration('initial_pose_y'),
                'initial_pose_theta':      LaunchConfiguration('initial_pose_theta'),
                'auto_set_initial_pose':   True,
                'initial_pose_delay_sec':  3.0,
            }]
        ),

        Node(
            package='summon',
            executable='summon_server',
            name='summon_server',
            output='screen',
        ),
    ])