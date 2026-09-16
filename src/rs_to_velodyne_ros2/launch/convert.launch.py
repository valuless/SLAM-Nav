from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rs_to_velodyne_ros2',
            executable='rs_to_velodyne_ros2',
            name='rs_converter',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_type': 'XYZIRT',
                'output_type': 'XYZIRT',
                'input_topic': '/rslidar_points',
                'output_topic': '/velodyne_points',
                'output_frame_id': 'rslidar'
            }]
        )
    ])