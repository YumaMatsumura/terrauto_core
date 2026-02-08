// Copyright 2026 Yuma Matsumura All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TERRAUTO_VOXEL_GRID_FILTER__VOXEL_GRID_FILTER_COMPONENT_HPP_
#define TERRAUTO_VOXEL_GRID_FILTER__VOXEL_GRID_FILTER_COMPONENT_HPP_

#include "terrauto_type_adapter/pcl_type_adapter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <memory>

namespace terrauto_voxel_grid_filter
{

class VoxelGridFilter : public rclcpp::Node
{
public:
  using PCLPointType = pcl::PointXYZI;
  using PCLPointCloudType = pcl::PointCloud<PCLPointType>;
  using PCLPointCloudTypePtr = std::shared_ptr<PCLPointCloudType>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloudAdaptedType = rclcpp::TypeAdapter<PCLPointCloudTypePtr, PointCloud2>;

  explicit VoxelGridFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VoxelGridFilter();

private:
  void pointsCallback(const PCLPointCloudTypePtr & cloud_ptr);

  rclcpp::Publisher<PointCloudAdaptedType>::SharedPtr pub_points_;
  rclcpp::Subscription<PointCloudAdaptedType>::SharedPtr sub_points_;

  double voxel_leaf_x_;
  double voxel_leaf_y_;
  double voxel_leaf_z_;
};

}  // namespace terrauto_voxel_grid_filter

#endif  // TERRAUTO_VOXEL_GRID_FILTER__VOXEL_GRID_FILTER_COMPONENT_HPP_
