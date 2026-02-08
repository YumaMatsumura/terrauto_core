cmake_minimum_required(VERSION 3.14)
project(terrauto_voxel_grid_filter)

set(voxel_grid_filter_executable voxel_grid_filter)
set(voxel_grid_filter_library ${voxel_grid_filter_executable}_core)

# find dependencies
find_package(terrauto_cmake REQUIRED)
terrauto_package()

# create the executor target
ament_auto_add_executable(${voxel_grid_filter_executable}
  src/voxel_grid_filter_node.cpp
  src/voxel_grid_filter_component.cpp
)

# create the component target
ament_auto_add_library(${voxel_grid_filter_library} SHARED
  src/voxel_grid_filter_component.cpp
)
rclcpp_components_register_nodes(${voxel_grid_filter_library}
  "terrauto_voxel_grid_filter::VoxelGridFilter"
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
