from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rslidar_sdk'),'launch','humble_start.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rs_to_velodyne_ros2'),'launch','convert.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('fast_lio'),'launch','mapping.launch.py')
            ),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('odom_to_tf_ros2'),'launch','odom_to_tf.launch.py')
        #     ),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('navigation'),'launch','rslidar_scan.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('dobot_atom_bridge'),'launch','atom_bridge.launch.py')
            ),
        ),
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'map': '/root/atom_ros2_ws/src/map/test.yaml',
                        'slam': 'True',
                        'autostart': 'True',
                        'use_sim_time': 'False',
                    }.items(),
                )
            ]
        ),
   ])