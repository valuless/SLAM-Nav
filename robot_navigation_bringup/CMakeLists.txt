cmake_minimum_required(VERSION 3.8)
project(robot_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/time_corrector.py
  DESTINATION lib/${PROJECT_NAME}/
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
