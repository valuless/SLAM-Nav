cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED)

add_library(pcd2pgm SHARED
  src/pcd2pgm.cpp
)

target_include_directories(pcd2pgm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pcd2pgm
  rclcpp
  rclcpp_components
)

target_include_directories(pcd2pgm PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd2pgm ${PCL_LIBRARIES})


rclcpp_components_register_node(pcd2pgm
  PLUGIN "pcd2pgm::Pcd2Pgm"
  EXECUTABLE pcd2pgm_node
)

install(
  TARGETS pcd2pgm pcd2pgm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
