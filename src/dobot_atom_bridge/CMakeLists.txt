cmake_minimum_required(VERSION 3.8)
project(dobot_atom_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 需要 C++17（SDK 使用 C++17 标准）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# ROS2 依赖
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(std_srvs REQUIRED)
find_package(dobot_atom REQUIRED)

# ============================================================
# dobot_atom_sdk 路径配置
# ============================================================
set(ATOM_SDK_DIR "${CMAKE_SOURCE_DIR}/../dobot_atom_sdk"
  CACHE PATH "Path to dobot_atom_sdk")

# SDK 头文件路径
set(ATOM_SDK_INCLUDE_DIRS
  ${ATOM_SDK_DIR}
  ${ATOM_SDK_DIR}/rpc
  ${ATOM_SDK_DIR}/common
  ${ATOM_SDK_DIR}/thirdparty
  ${ATOM_SDK_DIR}/thirdparty/nlohmann
)

# SDK RPC 库（仅 RPC 功能，无额外依赖）
set(ATOM_SDK_LIBS
  ${ATOM_SDK_DIR}/lib/libclient_rpc.a
)

# ============================================================
# 库文件
# ============================================================
add_library(${PROJECT_NAME} SHARED
  src/atom_bridge_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${ATOM_SDK_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  dobot_atom
)

target_link_libraries(${PROJECT_NAME}
  ${ATOM_SDK_LIBS}
)

# ============================================================
# 可执行文件
# ============================================================
add_executable(atom_bridge
  src/atom_bridge.cpp
)

target_link_libraries(atom_bridge
  ${PROJECT_NAME}
)

# ============================================================
# 安装
# ============================================================
install(TARGETS
  ${PROJECT_NAME}
  atom_bridge
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================================================
# 测试
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  dobot_atom
)

ament_package()
