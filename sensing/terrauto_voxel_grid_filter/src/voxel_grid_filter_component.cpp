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

#include "terrauto_voxel_grid_filter/voxel_grid_filter_component.hpp"

#include <memory>

namespace terrauto_voxel_grid_filter
{

VoxelGridFilter::VoxelGridFilter(const rclcpp::NodeOptions & options)
: Node("voxel_grid_filter_node", options)
{
  // Parameters
  this->declare_parameter<double>("voxel_leaf_x", 0.1);
  this->declare_parameter<double>("voxel_leaf_y", 0.1);
  this->declare_parameter<double>("voxel_leaf_z", 0.1);

  this->get_parameter("voxel_leaf_x", voxel_leaf_x_);
  this->get_parameter("voxel_leaf_y", voxel_leaf_y_);
  this->get_parameter("voxel_leaf_z", voxel_leaf_z_);

  // Publisher
  pub_points_ =
    this->create_publisher<PointCloudAdaptedType>("output_points", rclcpp::SensorDataQoS());

  // Callback Group
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  // Subscriber
  sub_points_ = this->create_subscription<PointCloudAdaptedType>(
    "input_points", rclcpp::SensorDataQoS(),
    std::bind(&VoxelGridFilter::pointsCallback, this, std::placeholders::_1), sensor_sub_opt);
}

VoxelGridFilter::~VoxelGridFilter()
{
}

void VoxelGridFilter::pointsCallback(const PCLPointCloudTypePtr & cloud_ptr)
{
  auto filtered_cloud_ptr = std::make_shared<PCLPointCloudType>();

  pcl::VoxelGrid<PCLPointType> voxel_grid_filter;

  voxel_grid_filter.setLeafSize(voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_);
  voxel_grid_filter.setInputCloud(cloud_ptr);
  voxel_grid_filter.filter(*filtered_cloud_ptr);

  pub_points_->publish(filtered_cloud_ptr);
}

}  // namespace terrauto_voxel_grid_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(terrauto_voxel_grid_filter::VoxelGridFilter)
