# Robot Navigation

基于 ROS 2 的机器人自主导航系统，集成了多线激光雷达、SLAM 定位、路径规划与机械臂控制功能。

## 系统架构

```
robot_nagivation/
└── src/
    ├── FAST_LIO/                  # LiDAR-惯性里程计 (SLAM)
    ├── navigation2/               # Nav2 导航框架
    │   ├── navigation2/           #   Nav2 元包
    │   ├── nav2_mppi_controller/  #   MPPI 模型预测路径积分控制器
    │   ├── nav2_regulated_pure_pursuit_controller/  #   规整纯追踪控制器
    │   ├── nav2_navfn_planner/    #   NavFn 全局规划器
    │   ├── nav2_dwb_controller/   #   DWB 局部控制器
    │   ├── nav2_waypoint_follower/#   航点跟随器
    │   ├── nav2_route/            #   路由规划
    │   └── nav2_voxel_grid/       #   体素栅格
    ├── navigation/                # 导航启动配置包 (pointcloud_to_laserscan + Nav2 + slam_toolbox)
    ├── pcd2pgm/                   # PCD 点云转 2D 占据栅格地图工具
    ├── rslidar_sdk/               # 速腾聚创 (RoboSense) LiDAR SDK
    ├── rslidar_msg/               # 速腾聚创 LiDAR 消息定义
    ├── rs_to_velodyne_ros2/       # 速腾 → Velodyne 点云格式转换
    ├── livox_ros_driver2/         # 览沃 (Livox) LiDAR ROS 2 驱动
    ├── Livox-SDK2/                # 览沃 LiDAR SDK
    ├── dobot_atom_sdk/            # 越疆 Dobot Atom 机械臂 SDK
    └── dobot_atom_bridge/         # Dobot Atom ROS 2 桥接节点
```

## 功能模块

### 感知 — LiDAR 驱动

支持两种主流激光雷达：

| 模块 | 厂商 | 说明 |
|------|------|------|
| `rslidar_sdk` + `rslidar_msg` | 速腾聚创 (RoboSense) | RS-LiDAR 驱动与消息定义 |
| `livox_ros_driver2` + `Livox-SDK2` | 览沃 (Livox) | Livox LiDAR ROS 2 驱动 |
| `rs_to_velodyne_ros2` | — | RoboSense 点云转为 Velodyne 格式，兼容下游算法 |

### 定位 — SLAM

| 模块 | 说明 |
|------|------|
| `FAST_LIO` | 紧耦合 LiDAR-IMU 迭代扩展卡尔曼滤波里程计，支持快速运动与退化环境下的鲁棒定位 |

### 导航 — Nav2

基于 Nav2 框架的完整导航栈：

| 模块 | 说明 |
|------|------|
| `navigation` | 集成启动包：`pointcloud_to_laserscan` → `slam_toolbox` → `nav2_bringup` |
| `nav2_mppi_controller` | MPPI 模型预测路径积分控制器，支持自适应避障，50+ Hz |
| `nav2_regulated_pure_pursuit_controller` | 规整化纯追踪控制器 |
| `nav2_navfn_planner` | 基于 NavFn 的全局路径规划器 |
| `nav2_dwb_controller` | DWB 局部轨迹规划器 |
| `nav2_waypoint_follower` | 航点序列跟随 |
| `nav2_route` | 路由级路径规划 |
| `nav2_voxel_grid` | 3D 体素栅格地图 |

### 建图 — 地图工具

| 模块 | 说明 |
|------|------|
| `pcd2pgm` | 将 `.pcd` 点云转为 `.pgm` + `.yaml` 2D 占据栅格地图，可直接用于 Nav2 `map_server` |

### 操作 — 机械臂

| 模块 | 说明 |
|------|------|
| `dobot_atom_sdk` | 越疆 Dobot Atom 机械臂 C++ SDK |
| `dobot_atom_bridge` | ROS 2 桥接节点，将 SDK 接口封装为 ROS 2 Topic/Service |

## 依赖环境

- **操作系统**: Ubuntu 22.04 (推荐)
- **ROS 2**: Humble Hawksbill
- **构建工具**: colcon, CMake 3.8+
- **数学库**: Eigen3, PCL
- **Linradar**: Livox SDK 2
- **机械臂**: Dobot Atom SDK

## 编译

```bash
# 安装 ROS 2 Humble (略)
# 安装依赖
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan ros-humble-navigation2 \
  libpcl-dev libeigen3-dev

# 克隆并编译
cd ~/robot_nagivation
colcon build --symlink-install
source install/setup.bash
```

## 运行示例

### 启动导航系统

```bash
ros2 launch navigation navigation.launch.py
```

### PCD 转 PGM 地图

```bash
ros2 launch pcd2pgm pcd2pgm.launch.py
```

### 速腾 LiDAR 转 Velodyne 格式

```bash
ros2 launch rs_to_velodyne_ros2 convert.launch.py
```

## License

本项目各子模块遵循其各自的许可证，详见各子目录下的 `LICENSE` 文件。
![输入图片说明](6afa79f3f3616dbbbccda39231256376.jpg)