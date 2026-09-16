# Robot Navigation

[English](README_en.md)

基于 ROS 2 的机器人自主导航工作空间，集成激光雷达驱动、激光惯性里程计、建图、Nav2、地图转换工具以及 Dobot Atom 机器人控制。

## 工作空间结构

```text
robot-navigation/
├── src/
│   ├── FAST_LIO/                 # 激光惯性里程计
│   ├── navigation2/              # Nav2 源码
│   ├── navigation/               # 项目启动文件与导航配置
│   ├── ground_reset/             # 地面平面标定与重置
│   ├── pcd2pgm/                  # PCD 转占据栅格地图
│   ├── rslidar_sdk/              # RoboSense 激光雷达 SDK 与 ROS 集成
│   ├── rslidar_msg/              # RoboSense 消息定义
│   ├── rs_to_velodyne_ros2/      # RoboSense 到 Velodyne 点云适配
│   ├── livox_ros_driver2/        # Livox ROS 2 驱动
│   ├── Livox-SDK2/               # Livox SDK
│   ├── dobot_atom/               # Dobot Atom ROS 2 消息定义
│   ├── dobot_atom_bridge/        # ROS 2 到 Dobot RPC 桥接
│   └── dobot_atom_sdk/           # 官方 SDK Git submodule
├── build/                        # colcon 生成，不纳入 Git
├── install/                      # colcon 生成，不纳入 Git
└── log/                          # colcon 生成，不纳入 Git
```

## 主要模块

| 领域 | 软件包 | 功能 |
|---|---|---|
| 激光雷达 | `rslidar_sdk`、`rslidar_msg` | RoboSense 激光雷达驱动与消息定义 |
| 激光雷达 | `livox_ros_driver2`、`Livox-SDK2` | Livox 激光雷达驱动与 SDK |
| 数据适配 | `rs_to_velodyne_ros2` | 将 RoboSense 点云转换为兼容 Velodyne 的格式 |
| 定位 | `FAST_LIO` | 紧耦合激光雷达—IMU 里程计 |
| 导航 | `navigation`、`navigation2` | 项目导航配置与 Nav2 导航栈 |
| 建图 | `pcd2pgm` | 将 PCD 地图转换为 PGM/YAML 占据栅格地图 |
| 标定 | `ground_reset` | 地面平面标定与位姿重置 |
| Dobot | `dobot_atom` | ROS 2 接口定义 |
| Dobot | `dobot_atom_bridge` | 将 ROS 2 速度、FSM 和上肢控制指令桥接到官方 SDK |
| Dobot | `dobot_atom_sdk` | 以 Git submodule 管理的 Dobot Atom 官方 SDK |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- `colcon` 与 `rosdep`
- CMake 3.8 或更高版本
- Eigen3 与 PCL
- 开发机能够通过网络访问所选激光雷达和 Dobot Atom 机器人

## 克隆与编译

克隆仓库并同时初始化 submodule：

```bash
git clone --recurse-submodules \
  https://gitee.com/valuless/robot-navigation.git
cd robot-navigation
```

如果克隆时没有初始化 submodule：

```bash
git submodule update --init --recursive
```

在工作空间根目录安装 ROS 依赖并编译：

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 使用示例

### 启动导航系统

```bash
ros2 launch navigation system.launch.py
```

### 将 PCD 地图转换为 PGM/YAML

```bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```

### 将 RoboSense 点云转换为 Velodyne 格式

```bash
ros2 launch rs_to_velodyne_ros2 convert.launch.py
```

### 启动 Dobot Atom Bridge

```bash
ros2 launch dobot_atom_bridge atom_bridge.launch.py \
  rpc_ip:=192.168.8.234 \
  rpc_port:=51234
```

Bridge 提供的主要 ROS 2 接口包括：

- `/cmd_vel`
- `/fsm_cmd`
- `/fsm_state`
- `/connection_state`
- `/upper_limb_cmd`
- `/upper_limb_service`

## 激光雷达配置

请根据实际连接的雷达选择对应驱动和 FAST-LIO YAML。其他型号的配置在点云格式兼容时可能也能启动，但扫描频率、时间戳单位、线数或外参不匹配时，定位精度可能下降且不一定立即报错。

FAST-LIO 的雷达配置位于 `src/FAST_LIO/config/`。

## 许可证

仓库中引入的上游项目继续遵循各自的许可证。重新分发前请查看各组件目录中的 `LICENSE` 文件。

![Robot Navigation 系统](6afa79f3f3616dbbbccda39231256376.jpg)
