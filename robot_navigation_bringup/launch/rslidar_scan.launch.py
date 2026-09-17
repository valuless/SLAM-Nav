from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 将 FAST-LIO 配准点云转换为 Nav2 使用的二维激光扫描
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.5,  # 500ms，兼容FAST_LIO 5Hz(~200ms)的TF发布频率
                'min_height': -1.4,
                'max_height': -0.2,
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.0174533,
                'range_min': 0.2,
                'range_max': 20.0,
                'scan_time': 0.1,
                'use_inf': True,
                'use_sim_time': False,
                'timestamp_unit': 1,  # 使用毫秒时间戳
            }],
            output='screen',
        ),
    ])
