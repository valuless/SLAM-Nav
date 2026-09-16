from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
        # 获取包路径
    pkg_share = get_package_share_directory('robot_navigation_bringup')
    
    return LaunchDescription([
        # 时间修正节点
        Node(
            package='robot_navigation_bringup',
            executable='time_corrector.py',
            name='time_corrector',
            output='screen',
        ),
        # 节点1：点云转激光
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered_fixed'),
                ('scan', '/scan'),
                ('/tf', '/tf_fixed'),  # 使用时间修正后的TF，与cloud时间戳一致
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
