rs_to_velodyne_ros2

ROS2 速腾RoboSense激光雷达点云转 Velodyne 标准格式转换包
功能简介

将速腾16线 / Ruby 128线激光雷达原始点云，转换为标准 Velodyne XYZI / XYZIR / XYZIRT 格式，适配 LIO-SAM、Cartographer、Fast-LIO2 等仅支持 Velodyne 点云格式的 SLAM 算法。
支持类型
输入格式

    XYZI：速腾普通点云（x,y,z,intensity）
    XYZIRT：速腾带环号+时间戳点云（x,y,z,intensity,ring,timestamp）

输出格式

    XYZI：基础坐标+强度
    XYZIR：坐标+强度+激光环号
    XYZIRT：坐标+强度+激光环号+点内相对时间戳（完全兼容Velodyne）

适配雷达

    RoboSense 16线 激光雷达
    RoboSense Ruby 128线 激光雷达

环境依赖

    ROS2 Humble / Iron / Rolling
    PCL
    rclcpp、sensor_msgs、pcl_conversions

编译安装

    cd ~/ros2_ws/src
    git clone https://github.com/valuless/rs_to_velodyne_ros2.git
    cd ..
    colcon build --packages-select rs_to_velodyne_ros2
    source install/setup.bash
    ros2 launch rs_to_velodyne_ros2 convert.launch.py