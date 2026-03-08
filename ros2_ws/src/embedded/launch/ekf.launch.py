import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ekf_config = os.path.join(
        get_package_share_directory('embedded'),
        'config',
        'ekf_params.yaml'
    )

    return LaunchDescription([

        # Fake odometry — replace with real Arduino bridge later
        Node(
            package='embedded',
            executable='fake_odometry',
            name='fake_odometry',
            output='screen',
        ),

        # EKF — fuses /odom + /imu/data → clean /odom + odom->base_link TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odom')
            ]
        ),
    ])