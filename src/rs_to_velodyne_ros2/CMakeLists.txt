cmake_minimum_required(VERSION 3.8)
project(rs_to_velodyne_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# 包含目录
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 链接目录
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# 创建可执行文件
add_executable(rs_to_velodyne_ros2 src/rs_to_velodyne_ros2.cpp)

# 设置依赖
ament_target_dependencies(rs_to_velodyne_ros2
  rclcpp
  sensor_msgs
  pcl_conversions
)

# 链接 PCL 库
target_link_libraries(rs_to_velodyne_ros2
  ${PCL_LIBRARIES}
)

# 安装
install(TARGETS rs_to_velodyne_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()