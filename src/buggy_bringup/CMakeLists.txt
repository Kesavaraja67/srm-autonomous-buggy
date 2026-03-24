cmake_minimum_required(VERSION 3.8)
project(buggy_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds rviz
install(DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
