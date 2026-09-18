cmake_minimum_required(VERSION 3.8)
project(robot_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
