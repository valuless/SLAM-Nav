"""机器人导航系统总启动入口。

职责：组合雷达驱动、点云格式转换、FAST-LIO、二维激光生成、底盘桥接、
RViz 和 Nav2。各组件的算法参数保存在 config/，本文件只管理启动顺序、
组件连接关系以及需要全系统保持一致的启动选项。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_lidar_pipeline(context, params_file):
    """在启动节点前校验转换器线数与 FAST-LIO 扫描线数一致。"""
    # OpaqueFunction 回调必须接收 LaunchContext；本校验不需要读取上下文。
    del context

    # lidar_pipeline.yaml 是标准 ROS 2 参数文件，这里只读取两个关联参数。
    with open(params_file, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)

    ring_count = params['rs_converter']['ros__parameters']['ring_count']
    scan_line = (
        params['laser_mapping']['ros__parameters']['preprocess']['scan_line'])

    # 0 是配置文件中的阻断占位值，必须在实机探测线数后修改。
    if not isinstance(ring_count, int) or ring_count <= 0:
        raise RuntimeError(
            'Set rs_converter.ros__parameters.ring_count in '
            f'{params_file} after running ring_probe.py')
    # 转换器输出的 ring 范围必须与 FAST-LIO 预处理线数完全一致。
    if ring_count != scan_line:
        raise RuntimeError(
            f'LiDAR ring_count ({ring_count}) and FAST-LIO scan_line '
            f'({scan_line}) must match in {params_file}')
    return []


def generate_launch_description():
    """构造完整系统的 ROS 2 LaunchDescription。"""
    # 从安装空间定位 bringup 包，避免依赖源码工作区的绝对路径。
    bringup_share = get_package_share_directory('robot_navigation_bringup')
    config_dir = os.path.join(bringup_share, 'config')

    # 系统级启动参数。LaunchConfiguration 在 launch 执行阶段解析，允许命令行覆盖。
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')               # 已有二维地图 YAML 路径。
    slam = LaunchConfiguration('slam')                  # true 建图，false 定位。
    autostart = LaunchConfiguration('autostart')        # Nav2 生命周期自动激活。
    rviz = LaunchConfiguration('rviz')                  # 是否启动系统唯一 RViz。
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')  # Nav2 延迟秒数。

    # 所有部署配置集中安装到 robot_navigation_bringup/config。
    # rslidar.yaml 使用 RoboSense SDK 原生格式，其余 YAML 使用 ROS 2 参数格式。
    rslidar_config = os.path.join(config_dir, 'rslidar.yaml')
    lidar_pipeline_config = os.path.join(config_dir, 'lidar_pipeline.yaml')
    laserscan_config = os.path.join(config_dir, 'laserscan.yaml')
    nav2_config = os.path.join(config_dir, 'nav2_params.yaml')
    rviz_config = os.path.join(config_dir, 'test.rviz')

    return LaunchDescription([
        # false 使用系统时钟；true 使用仿真或 rosbag 发布的 /clock。
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the ROS simulation clock'),
        # slam=false 时必须传入地图 YAML 的绝对路径；slam=true 时允许为空。
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Absolute map YAML path; required when slam is false'),
        # true 启动 SLAM Toolbox；false 启动地图服务器与 AMCL 定位。
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Start SLAM instead of map-based localization'),
        # true 让 Nav2 lifecycle manager 自动 configure 并 activate 各节点。
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes'),
        # 子系统自带的 RViz 均被关闭，只由此开关管理一个统一实例。
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start one RViz instance from the bringup configuration'),
        # 给传感器、里程计和 TF 链预留初始化时间，单位为秒。
        DeclareLaunchArgument(
            'nav2_start_delay',
            default_value='5.0',
            description='Seconds to wait before starting Nav2'),

        # 在创建任何依赖点云线数的节点前执行配置一致性检查。
        OpaqueFunction(
            function=validate_lidar_pipeline,
            kwargs={'params_file': lidar_pipeline_config}),

        # 启动 RoboSense 驱动。驱动参数通过其原生 config_path 传入；关闭子包 RViz。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rslidar_sdk'),
                'launch',
                'humble_start.py')),
            launch_arguments={
                'config_path': rslidar_config,
                'rviz': 'false',
            }.items(),
        ),

        # 将 RoboSense PointCloud2 转为 FAST-LIO 可读取的 Velodyne XYZIRT 布局。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rs_to_velodyne_ros2'),
                'launch',
                'convert.launch.py')),
            launch_arguments={
                'params_file': lidar_pipeline_config,
            }.items(),
        ),

        # 启动 FAST-LIO 融合雷达和 IMU；与转换器共用 lidar_pipeline.yaml。
        # 关闭 FAST-LIO 自带 RViz，避免整机启动时出现多个窗口。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('fast_lio'),
                'launch',
                'mapping.launch.py')),
            launch_arguments={
                'config_path': config_dir,
                'config_file': 'lidar_pipeline.yaml',
                'use_sim_time': use_sim_time,
                'rviz': 'false',
            }.items(),
        ),

        # 将 FAST-LIO 配准点云投影成 Nav2 使用的二维 LaserScan。
        # cloud_in/scan 是该节点的标准相对话题，重映射为系统绝对话题。
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
            parameters=[
                laserscan_config,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool),
                },
            ],
            output='screen',
        ),

        # 启动 Dobot Atom 底盘桥接，将 Nav2 /cmd_vel 转交给机器人控制接口。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('dobot_atom_bridge'),
                'launch',
                'atom_bridge.launch.py')),
        ),

        # 系统只启动这一份 RViz，并加载 bringup 提供的统一显示配置。
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
            condition=IfCondition(rviz),
            output='screen',
        ),

        # 延迟启动 Nav2，避免传感器、FAST-LIO 和 TF 尚未就绪时生命周期激活失败。
        TimerAction(
            period=nav2_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'bringup_launch.py')),
                    launch_arguments={
                        'map': map_file,              # 定位模式使用的地图 YAML。
                        'params_file': nav2_config,   # Nav2 与 AMCL 参数文件。
                        'slam': slam,                 # 建图/定位模式选择。
                        'autostart': autostart,       # 生命周期自动激活开关。
                        'use_sim_time': use_sim_time,  # 与全系统统一时钟源。
                    }.items(),
                ),
            ],
        ),
    ])
