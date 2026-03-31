import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    wb_navigation_dir = get_package_share_directory('navigation')
    nav2_bringup_dir  = get_package_share_directory('nav2_bringup')
    wb_description_dir = get_package_share_directory('description')

    map_file   = os.path.join(wb_navigation_dir, 'maps', 'blank_map.yaml')
    nav2_params = os.path.join(wb_navigation_dir, 'config', 'nav2_params.yaml')
    urdf_file  = os.path.join(wb_description_dir, 'urdf', 'warehouse_bot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        # Robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # Fake sensors
        Node(
            package='perception',
            executable='fake_laser_scan',
            name='fake_laser_scan',
            output='screen',
        ),
        Node(
            package='perception',
            executable='fake_imu',
            name='fake_imu',
            output='screen',
        ),

        # Fake odometry + EKF
        Node(
            package='embedded',
            executable='fake_odometry',
            name='fake_odometry',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('embedded'),
                'config', 'ekf_params.yaml'
            )],
            remappings=[('odometry/filtered', '/odom')]
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map':          map_file,
                'use_sim_time': 'false',
                'params_file':  nav2_params,
                'autostart':    'true',
            }.items()
        ),
    ])