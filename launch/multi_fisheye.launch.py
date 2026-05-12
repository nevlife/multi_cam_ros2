import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('multi_cam'),
        'config',
        'multi_fisheye.yaml',
    )

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to YAML config file with camera parameters',
        ),

        Node(
            package='multi_cam',
            executable='multi_fisheye_node',
            name='multi_fisheye_pub',
            output='screen',
            parameters=[config_file],
        ),
    ])
