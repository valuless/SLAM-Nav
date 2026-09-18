import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rslidar_sdk')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(package_share, 'config', 'config.yaml'),
            description='RoboSense SDK YAML configuration'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start the RoboSense RViz configuration'),
        Node(
            namespace='rslidar_sdk',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            parameters=[{'config_path': LaunchConfiguration('config_path')}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', os.path.join(package_share, 'rviz', 'rviz2.rviz'),
            ],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
