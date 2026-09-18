import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('rs_to_velodyne_ros2'),
        'config',
        'converter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Converter ROS parameter file'),
        Node(
            package='rs_to_velodyne_ros2',
            executable='rs_to_velodyne_ros2',
            name='rs_converter',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
