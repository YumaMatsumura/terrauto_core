cmake_minimum_required(VERSION 3.14)
project(terrauto_ndt_scan_matcher)

set(ndt_scan_matcher_executable ndt_scan_matcher)
set(ndt_scan_matcher_library ${ndt_scan_matcher_executable}_core)

# cuPCL paths
set(CUNDT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/cuPCL/cuNDT)
set(CUNDT_INCLUDE_DIRS ${CUNDT_ROOT}/lib)
set(CUNDT_LIB_PATH ${CUNDT_ROOT}/lib/libcudandt.so)

# find dependencies
find_package(terrauto_cmake REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)
terrauto_package()

# sanity check
if(NOT EXISTS "${CUNDT_LIB_PATH}")
  message(FATAL_ERROR "libcudandt.so not found: ${CUNDT_LIB_PATH}")
endif()

# imported target for libcudandt.so
add_library(cudandt SHARED IMPORTED GLOBAL)
set_target_properties(cudandt PROPERTIES
  IMPORTED_LOCATION "${CUNDT_LIB_PATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${CUNDT_INCLUDE_DIRS}"
)

# create the executor target
ament_auto_add_executable(${ndt_scan_matcher_executable}
  src/ndt_scan_matcher_node.cpp
  src/ndt_scan_matcher_component.cpp
  src/utils.cpp
)
target_include_directories(${ndt_scan_matcher_executable} PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  ${CUDA_INCLUDE_DIRS}
  ${CUNDT_INCLUDE_DIRS}
)
target_link_libraries(${ndt_scan_matcher_executable}
  ${CUDA_LIBRARIES}
  cudandt
)

# create the component target
ament_auto_add_library(${ndt_scan_matcher_library} SHARED
  src/ndt_scan_matcher_component.cpp
  src/utils.cpp
)
target_include_directories(${ndt_scan_matcher_library} PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  ${CUDA_INCLUDE_DIRS}
  ${CUNDT_INCLUDE_DIRS}
)
target_link_libraries(${ndt_scan_matcher_library}
  ${CUDA_LIBRARIES}
  cudandt
)

# Ensure runtime loader finds libcudandt.so in the same install dir (install/.../lib)
set_target_properties(${ndt_scan_matcher_library} PROPERTIES
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
)

rclcpp_components_register_nodes(${ndt_scan_matcher_library}
  "terrauto_ndt_scan_matcher::NdtScanMatcher"
)

# Install dependent .so into this package's install/lib
install(FILES "${CUNDT_LIB_PATH}" DESTINATION lib)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
