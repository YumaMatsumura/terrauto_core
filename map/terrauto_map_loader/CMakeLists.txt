cmake_minimum_required(VERSION 3.14)
project(terrauto_map_loader)

set(map_loader_executable map_loader)
set(map_loader_library ${map_loader_executable}_core)

# find dependencies
find_package(terrauto_cmake REQUIRED)
terrauto_package()

# create the executor target
ament_auto_add_executable(${map_loader_executable}
  src/map_loader_node.cpp
  src/map_loader_component.cpp
)

# create the component target
ament_auto_add_library(${map_loader_library} SHARED
  src/map_loader_component.cpp
)
rclcpp_components_register_nodes(${map_loader_library}
  "terrauto_map_loader::MapLoader"
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
