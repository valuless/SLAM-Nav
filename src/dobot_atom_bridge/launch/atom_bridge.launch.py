"""
Dobot Atom Bridge 启动文件

使用 dobot_atom_sdk 将机器人 RPC/DDS 接口桥接到 ROS2。

用法:
  ros2 launch dobot_atom_bridge atom_bridge.launch.py
  ros2 launch dobot_atom_bridge atom_bridge.launch.py rpc_ip:=192.168.1.100 rpc_port:=51234
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='atom_bridge_node',
        description='ROS2 node name'
    )

    rpc_ip_arg = DeclareLaunchArgument(
        'rpc_ip', default_value='192.168.8.234',
        description='Robot RPC server IP address'
    )

    rpc_port_arg = DeclareLaunchArgument(
        'rpc_port', default_value='51234',
        description='Robot RPC server port'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='ROS2 log level'
    )

    # 桥接节点
    bridge_node = Node(
        package='dobot_atom_bridge',
        executable='atom_bridge',
        name=LaunchConfiguration('node_name'),
        output='screen',
        arguments=[
            '--rpc-ip', LaunchConfiguration('rpc_ip'),
            '--rpc-port', LaunchConfiguration('rpc_port'),
            '--ros-args', '--log-level', LaunchConfiguration('log_level'),
        ],
        remappings=[
            # Nav2 controller 发布 /cmd_vel，桥接节点默认监听 ~/cmd_vel
            # 这条 remap 让两者连通
            ('~/cmd_vel', '/cmd_vel'),
            # 将相对话题映射到全局，方便外部节点直接使用
            ('~/fsm_cmd', '/fsm_cmd'),
            ('~/fsm_state', '/fsm_state'),
            ('~/connection_state', '/connection_state'),
            ('~/upper_limb_cmd', '/upper_limb_cmd'),
            ('~/upper_limb_service', '/upper_limb_service'),
        ],
    )

    return LaunchDescription([
        node_name_arg,
        rpc_ip_arg,
        rpc_port_arg,
        log_level_arg,
        bridge_node,
    ])
