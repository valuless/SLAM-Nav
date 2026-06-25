import os
import launch
import launch_ros

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    pcd2pgm_config = LaunchConfiguration(
        'pcd2pgm_config',
        default=os.path.join(
            get_package_share_directory('pcd2pgm'),
            'config',
            'config_pcd2pgm.yaml'
        )
    )

    pcd2pgm_node = Node(
        package='pcd2pgm',
        executable='pcd2pgm_node',
        name='pcd2pgm_node',
        output='screen',
        parameters=[pcd2pgm_config]
    )

    ld.add_action(pcd2pgm_node)

    return ld