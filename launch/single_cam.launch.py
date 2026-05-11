import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('multi_fisheye'),
        'config',
        'c922.yaml',
    )

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML config (single camera params)',
        ),

        Node(
            package='multi_fisheye',
            executable='single_cam_node',
            name='single_cam_pub',
            output='screen',
            parameters=[config_file],
        ),
    ])
