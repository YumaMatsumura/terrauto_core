cmake_minimum_required(VERSION 3.14)
project(terrauto_bringup)

# find dependencies
find_package(terrauto_cmake REQUIRED)
terrauto_package()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch maps params rviz
)
