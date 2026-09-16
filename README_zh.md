# SLAM-Nav

[English](README_en.md)

这是一个面向真实移动机器人平台的 ROS 2 自主导航工作区。它不是单一算法包，而是把激光雷达驱动、点云格式适配、LiDAR-IMU 建图定位、Nav2 路径规划与 Dobot Atom 机器人控制桥接组织在一起，用于完成从传感器数据采集到机器人执行运动指令的完整链路。

项目的核心目标是让机器人能够在实际环境中完成地图构建、定位、避障、路径规划和速度控制，并为后续接入机械臂、上肢控制或任务级调度保留接口。

## 项目定位

本工作区适合以下场景：

- 基于 ROS 2 Humble 搭建移动机器人导航系统。
- 使用 RoboSense 或 Livox 激光雷达作为主要感知输入。
- 使用 FAST-LIO 进行 LiDAR-IMU 里程计和点云建图。
- 使用 Nav2 完成地图导航、局部避障、路径跟踪和任务执行。
- 将 Nav2 输出的 `/cmd_vel` 对接到 Dobot Atom 机器人底盘或控制接口。
- 将三维点云地图转换为二维占据栅格地图，用于 Nav2 `map_server`。

整体设计更偏向工程集成：各个开源组件和厂商 SDK 保持相对独立，项目通过自定义启动包、桥接节点和话题约定把它们串成一条可运行的数据链路。

## 总体架构

工作区以 `src/` 下的 ROS 2 package 为主体，主要分为五层：

```text
传感器与驱动层
  RoboSense: rslidar_sdk + rslidar_msg
  Livox:     Livox-SDK2 + livox_ros_driver2

点云适配层
  rs_to_velodyne_ros2

建图与定位层
  FAST_LIO
  pcd2pgm

导航决策层
  robot_navigation_bringup
  navigation2

机器人执行层
  dobot_atom
  dobot_atom_bridge
```

典型运行链路如下：

```text
LiDAR / IMU
   |
   v
雷达驱动节点发布点云
   |
   v
点云格式转换与时间修正
   |
   v
FAST_LIO 输出里程计、TF 和配准点云
   |
   +--> pointcloud_to_laserscan 生成 /scan
   |
   v
Nav2 根据地图、TF、/scan 和目标点生成 /cmd_vel
   |
   v
dobot_atom_bridge 将 /cmd_vel 转换为机器人控制接口调用
```

其中 `robot_navigation_bringup` 包承担系统级编排职责，它不是算法本身，而是把雷达驱动、格式转换、FAST-LIO、点云转激光、Nav2 和 Dobot 桥接节点按顺序拉起来。

## 主要目录

```text
src/
├── dobot_atom/              # Dobot Atom ROS 消息定义
├── dobot_atom_bridge/       # Dobot Atom 与 ROS 2 的控制桥接
├── dobot_atom_sdk/          # Dobot Atom 官方 SDK Git submodule
├── FAST_LIO/                # LiDAR-IMU 里程计与点云建图
├── ground_reset/            # 地面标定与辅助脚本
├── Livox-SDK2/              # Livox 官方 SDK
├── livox_ros_driver2/       # Livox ROS 驱动
├── navigation2/             # Nav2 导航框架源码及相关功能包
├── pcd2pgm/                 # PCD 点云到 PGM/YAML 占据栅格地图转换
├── robot_navigation_bringup/ # 本项目的系统启动与导航配置入口
├── rs_to_velodyne_ros2/     # RoboSense 点云转 Velodyne 风格点云
├── rslidar_msg/             # RoboSense 雷达消息定义
└── rslidar_sdk/             # RoboSense 雷达驱动
```

## 模块说明

### `robot_navigation_bringup`

这是本项目最重要的集成包，负责组织整套导航系统的启动与配置。

它包含：

- `launch/system.launch.py`：系统级启动入口，组合雷达驱动、点云转换、FAST-LIO、激光扫描生成、Dobot 桥接和 Nav2。
- `launch/rslidar_scan.launch.py`：将 FAST-LIO 输出的配准点云转换为 `/scan`，供 Nav2 局部代价地图使用。
- `config/nav2_params.yaml`：Nav2 参数配置，包括控制器、代价地图、行为树、规划器等。
- `config/amcl_params.yaml`：AMCL 相关定位参数。
- `scripts/time_corrector.py`：用于处理 TF 或点云链路中的时间同步问题。

从职责上看，`robot_navigation_bringup` 是“系统装配层”。如果需要调整话题名、地图路径、机器人速度限制、局部代价地图大小、避障距离或控制器参数，通常优先从这个包开始看。

### `FAST_LIO`

`FAST_LIO` 负责将 LiDAR 与 IMU 数据融合，输出稳定的里程计、位姿估计、TF 和配准后的点云。它在系统中承担两个角色：

- 建图：运行过程中积累环境点云，用于生成三维地图。
- 定位：为导航栈提供机器人当前位姿和坐标变换关系。

在当前链路中，FAST-LIO 的输出还会被转换成二维 LaserScan，使 Nav2 可以使用传统二维代价地图进行局部避障。

### `navigation2`

`navigation2` 是 ROS 2 的 Nav2 导航框架源码集合，包含全局规划、局部控制、行为树、代价地图、地图服务、生命周期管理、RViz 插件等组件。

本项目主要使用它来完成：

- 接收导航目标。
- 基于地图生成全局路径。
- 根据实时障碍物生成局部运动指令。
- 管理导航行为树和恢复行为。
- 输出 `/cmd_vel` 给底盘控制桥接层。

当前配置中局部控制器以 DWB Local Planner 为主，适合差速或类差速移动机器人。后续如果需要更平滑或更激进的轨迹跟踪，可以考虑切换或调试 MPPI、Regulated Pure Pursuit 等控制器。

### `rslidar_sdk`、`rslidar_msg` 与 `rs_to_velodyne_ros2`

这几个包组成 RoboSense 雷达输入链路。

`rslidar_sdk` 负责和雷达通信并发布原始点云，`rslidar_msg` 提供相关消息定义。由于很多 SLAM 算法对 Velodyne 风格点云字段更友好，`rs_to_velodyne_ros2` 用来把 RoboSense 点云转换成更通用的 `XYZI`、`XYZIR` 或 `XYZIRT` 格式。

当前转换节点的默认关系是：

```text
/rslidar_points  -->  rs_to_velodyne_ros2  -->  /velodyne_points
```

这使 RoboSense 雷达可以接入原本偏向 Velodyne 输入格式的建图或定位算法。

### `livox_ros_driver2` 与 `Livox-SDK2`

这两个目录提供 Livox 雷达支持。它们和 RoboSense 链路并列存在，说明这个工作区希望兼容不同雷达硬件。

如果实际平台使用 Livox，需要重点检查：

- Livox 雷达 IP 和主机网卡配置。
- `livox_ros_driver2/config/` 下的设备配置。
- FAST-LIO 中对应的雷达型号参数。
- 点云话题名是否与后续节点匹配。

### `pcd2pgm`

`pcd2pgm` 用于把三维 `.pcd` 点云地图转换成 Nav2 可直接加载的二维地图文件：

```text
map.pcd  -->  map.pgm + map.yaml
```

它的作用不是在线导航，而是离线地图生产。一般流程是先用 FAST-LIO 建出三维点云地图，再通过 `pcd2pgm` 提取地面、障碍物和占据区域，最终生成供 `nav2_map_server` 使用的二维占据栅格地图。

### `dobot_atom` 与 `dobot_atom_bridge`

`dobot_atom` 定义 Dobot Atom 相关 ROS 消息，`dobot_atom_bridge` 负责把 ROS 2 话题和机器人 RPC/DDS 控制接口连接起来。

桥接层的核心意义是把导航系统的通用速度指令转换成机器人平台能理解的控制命令：

```text
Nav2 /cmd_vel  -->  dobot_atom_bridge  -->  Dobot Atom RPC/DDS
```

`dobot_atom_bridge` 还提供状态、FSM、连接状态和上肢控制相关话题或服务，是移动导航与机器人本体之间的边界层。

## 关键话题与坐标系

常见话题关系如下：

| 类型 | 话题 | 说明 |
| --- | --- | --- |
| 原始点云 | `/rslidar_points` | RoboSense 驱动输出 |
| 适配点云 | `/velodyne_points` | 转换后的 Velodyne 风格点云 |
| 配准点云 | `/cloud_registered_fixed` | FAST-LIO 或时间修正后用于生成扫描的数据 |
| 激光扫描 | `/scan` | Nav2 局部代价地图输入 |
| 速度指令 | `/cmd_vel` | Nav2 输出，桥接到底盘 |
| 状态反馈 | `/connection_state`、`/fsm_state` | Dobot 桥接状态 |

常见坐标系包括：

- `map`：全局地图坐标系。
- `Odometry`：里程计坐标系，当前配置中局部代价地图使用它作为全局参考。
- `base_link`：机器人本体坐标系。
- `rslidar`：雷达坐标系。

在真实机器人上调试时，TF 连通性通常比单个节点是否启动更关键。若 Nav2 不动、代价地图为空或 RViz 中数据错位，应优先检查 `map -> Odometry -> base_link -> lidar` 这条 TF 链路是否稳定。

## 推荐理解路径

如果是第一次接触这个工作区，建议按下面顺序阅读和调试：

1. 先看 `src/robot_navigation_bringup/launch/system.launch.py`，理解系统启动了哪些模块。
2. 再看 `src/robot_navigation_bringup/launch/rslidar_scan.launch.py`，理解 `/scan` 如何从点云生成。
3. 查看 `src/robot_navigation_bringup/config/nav2_params.yaml`，理解 Nav2 控制器、代价地图和行为树配置。
4. 查看 `src/dobot_atom_bridge/launch/atom_bridge.launch.py`，理解 `/cmd_vel` 如何进入机器人平台。
5. 根据实际雷达型号，分别检查 `rslidar_sdk` 或 `livox_ros_driver2` 的配置。
6. 如果需要离线地图，再查看 `pcd2pgm` 的参数和输出路径。

这样读会比直接从 Nav2 或 FAST-LIO 源码开始更清晰，因为本项目真正的工程逻辑主要体现在“模块如何连接”。

## 环境假设

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- PCL
- Eigen3
- Nav2
- slam_toolbox
- pointcloud_to_laserscan
- 对应雷达厂商 SDK 与网络配置

基础依赖可以参考：

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  libpcl-dev \
  libeigen3-dev
```

实际部署时还需要根据雷达型号、机器人网络、Dobot Atom 控制接口地址补充配置。

## 克隆、构建与启动

克隆仓库时需要同时初始化 Dobot Atom SDK submodule：

```bash
git clone --recurse-submodules git@github.com:valuless/SLAM-Nav.git
```

建议将仓库克隆到 ROS 2 工作空间的 `src/` 目录，然后回到工作空间根目录执行构建。

在工作区根目录构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

启动集成系统：

```bash
ros2 launch robot_navigation_bringup system.launch.py
```

单独启动 Dobot Atom 桥接：

```bash
ros2 launch dobot_atom_bridge atom_bridge.launch.py
```

指定机器人 RPC 地址：

```bash
ros2 launch dobot_atom_bridge atom_bridge.launch.py rpc_ip:=192.168.8.234 rpc_port:=51234
```

单独运行 RoboSense 点云转换：

```bash
ros2 launch rs_to_velodyne_ros2 convert.launch.py
```

运行 PCD 到二维地图转换：

```bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```

这些命令只是入口。实际机器人上能否稳定运行，取决于雷达网络、IMU 数据、TF、地图路径、Nav2 参数和底盘控制接口是否一致。

## 调试重点

### 1. 雷达是否有数据

先确认原始点云话题是否发布，再检查转换后的点云话题。没有点云时，后续 FAST-LIO、点云转扫描和 Nav2 都无法正常工作。

### 2. TF 是否连续

导航系统依赖稳定的 TF。建议重点检查：

```text
map
└── Odometry
    └── base_link
        └── lidar frame
```

如果 TF 时间戳不一致，可以关注 `robot_navigation_bringup/scripts/time_corrector.py` 和 `rslidar_scan.launch.py` 中的 remapping。

### 3. `/scan` 是否合理

Nav2 当前通过 `pointcloud_to_laserscan` 使用二维扫描数据构建局部代价地图。若机器人不避障，应检查：

- `/cloud_registered_fixed` 是否存在。
- `/scan` 是否存在。
- `min_height` 和 `max_height` 是否截取到了正确高度范围。
- `target_frame` 是否能转换到 `base_link`。

### 4. Nav2 是否收到地图与目标

如果 Nav2 已启动但不能规划，通常需要检查地图文件路径、全局代价地图、局部代价地图、初始位姿和目标点。

当前 `system.launch.py` 中地图路径默认值带有现场部署特征，迁移机器或工作区后需要通过 launch 参数覆盖。

### 5. `/cmd_vel` 是否进入机器人

Nav2 只负责输出速度指令，真实机器人是否运动取决于 `dobot_atom_bridge` 是否连接成功，以及 `/cmd_vel` 是否正确 remap 到桥接节点监听的话题。

## 配置修改建议

常见修改入口：

| 需求 | 优先查看 |
| --- | --- |
| 修改系统启动顺序 | `robot_navigation_bringup/launch/system.launch.py` |
| 修改点云转 `/scan` 的高度范围 | `robot_navigation_bringup/launch/rslidar_scan.launch.py` |
| 修改 Nav2 控制器和速度限制 | `robot_navigation_bringup/config/nav2_params.yaml` |
| 修改地图路径 | `robot_navigation_bringup/launch/system.launch.py` |
| 修改 Dobot Atom IP 和端口 | `dobot_atom_bridge/launch/atom_bridge.launch.py` |
| 修改 RoboSense 点云格式 | `rs_to_velodyne_ros2/launch/convert.launch.py` |
| 生成二维地图 | `pcd2pgm/config/config_pcd2pgm.yaml` |

## 开发注意事项

- 不同雷达链路不要同时假设使用同一个点云话题，接入新雷达时应先统一话题名和 frame id。
- Nav2 对 TF、时间戳和 frame id 非常敏感，修改点云或里程计链路后要同步检查配置。
- `system.launch.py` 中部分默认值带有现场部署特征，迁移环境时应优先通过 launch 参数覆盖。
- 各第三方模块保留各自许可证和原始结构，修改时尽量把集成逻辑放在本项目自有包中。

## License

本工作区包含多个第三方组件和厂商 SDK。各子模块的许可证以其目录下的 `LICENSE`、`README` 或上游项目声明为准。本项目新增的集成代码应遵循对应 package 中声明的许可证。
