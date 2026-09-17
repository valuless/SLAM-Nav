from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ring_count',
            description='Physical LiDAR channel count; confirm with ring_probe.py'),
        Node(
            package='rs_to_velodyne_ros2',
            executable='rs_to_velodyne_ros2',
            name='rs_converter',
            output='screen',
            parameters=[{
                'input_topic': '/rslidar_points',
                'output_topic': '/velodyne_points',
                'output_type': 'XYZIRT',
                'output_frame_id': ParameterValue('', value_type=str),
                'ring_source': 'field',
                'ring_count': ParameterValue(
                    LaunchConfiguration('ring_count'), value_type=int),
                'time_source': 'timestamp',
                'time_mode': 'absolute',
                'time_unit': 'second',
                'time_order_policy': 'validate',
                'velodyne_layout': True,
                'pool_size': 1,
                'max_scan_period': 1.0,
            }],
        )
    ])
