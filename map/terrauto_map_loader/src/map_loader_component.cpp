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

#include "terrauto_map_loader/map_loader_component.hpp"

#include <memory>
#include <string>
#include <utility>

namespace
{
constexpr int PCL_LOAD_SUCCESS = 0;
constexpr int PCL_LOAD_FAILURE = -1;
}  // namespace

namespace terrauto_map_loader
{

MapLoader::MapLoader(const rclcpp::NodeOptions & options) : Node("map_loader_node", options)
{
  // Parameters
  this->declare_parameter<bool>("use_voxel_grid_filter", true);
  this->declare_parameter<double>("voxel_leaf_x", 0.1);
  this->declare_parameter<double>("voxel_leaf_y", 0.1);
  this->declare_parameter<double>("voxel_leaf_z", 0.1);
  this->declare_parameter<double>("translation_x", 0.0);
  this->declare_parameter<double>("translation_y", 0.0);
  this->declare_parameter<double>("translation_z", 0.0);
  this->declare_parameter<double>("rotation_roll", 0.0);
  this->declare_parameter<double>("rotation_pitch", 0.0);
  this->declare_parameter<double>("rotation_yaw", 0.0);
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<std::string>("map_file", "");
  this->declare_parameter<std::string>("file_type", "pcd");

  this->get_parameter("use_voxel_grid_filter", use_voxel_grid_filter_);
  this->get_parameter("voxel_leaf_x", voxel_leaf_x_);
  this->get_parameter("voxel_leaf_y", voxel_leaf_y_);
  this->get_parameter("voxel_leaf_z", voxel_leaf_z_);
  this->get_parameter("translation_x", translation_x_);
  this->get_parameter("translation_y", translation_y_);
  this->get_parameter("translation_z", translation_z_);
  this->get_parameter("rotation_roll", rotation_roll_);
  this->get_parameter("rotation_pitch", rotation_pitch_);
  this->get_parameter("rotation_yaw", rotation_yaw_);
  this->get_parameter("global_frame_id", global_frame_id_);
  this->get_parameter("map_file", map_file_);
  this->get_parameter("file_type", file_type_);

  // Publisher
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Timer
  timer_ =
    this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapLoader::timerCallback, this));
}

void MapLoader::timerCallback()
{
  // Publish once
  timer_->cancel();

  if (map_file_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'map_file' is empty. Please set a .pcd path.");
    return;
  }

  // Load Map
  auto cloud_ptr = std::make_shared<PCLPointCloudType>();
  int load_result = PCL_LOAD_FAILURE;
  if (file_type_ == "pcd") {
    load_result = pcl::io::loadPCDFile<PCLPointType>(map_file_, *cloud_ptr);
  } else if (file_type_ == "ply") {
    load_result = pcl::io::loadPLYFile<PCLPointType>(map_file_, *cloud_ptr);
  }

  if (load_result != PCL_LOAD_SUCCESS) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to load point cloud file: %s (PCL error code: %d)",
      map_file_.c_str(), load_result);
    return;
  }

  if (cloud_ptr->empty()) {
    RCLCPP_WARN(this->get_logger(), "Loaded point cloud is empty: %s", map_file_.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Successfully loaded %zu points from: %s", cloud_ptr->size(),
      map_file_.c_str());
  }

  auto transformed_cloud_ptr = transformPointCloud(cloud_ptr);
  if (use_voxel_grid_filter_) {
    transformed_cloud_ptr = voxelGridFilter(transformed_cloud_ptr);
  }

  auto out_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*transformed_cloud_ptr, *out_msg_ptr);

  out_msg_ptr->header.stamp = this->get_clock()->now();
  out_msg_ptr->header.frame_id = global_frame_id_;
  pub_map_->publish(std::move(out_msg_ptr));
}

MapLoader::PCLPointCloudTypePtr MapLoader::voxelGridFilter(
  const PCLPointCloudTypePtr & cloud_ptr) const
{
  if (!cloud_ptr || cloud_ptr->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input cloud for voxel grid filter is null or empty");
    return std::make_shared<PCLPointCloudType>();
  }

  if (voxel_leaf_x_ <= 0.0 || voxel_leaf_y_ <= 0.0 || voxel_leaf_z_ <= 0.0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid voxel leaf size (must be > 0): %.3f, %.3f, %.3f", voxel_leaf_x_,
      voxel_leaf_y_, voxel_leaf_z_);
    return cloud_ptr;
  }

  auto filtered_cloud_ptr = std::make_shared<PCLPointCloudType>();
  filtered_cloud_ptr->header = cloud_ptr->header;

  pcl::VoxelGrid<PCLPointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(
    static_cast<float>(voxel_leaf_x_), static_cast<float>(voxel_leaf_y_),
    static_cast<float>(voxel_leaf_z_));
  voxel_grid_filter.setInputCloud(cloud_ptr);
  voxel_grid_filter.filter(*filtered_cloud_ptr);

  RCLCPP_DEBUG(
    this->get_logger(), "Voxel grid filter: %zu -> %zu points (leaf size: %.3f, %.3f, %.3f)",
    cloud_ptr->size(), filtered_cloud_ptr->size(), voxel_leaf_x_, voxel_leaf_y_, voxel_leaf_z_);

  return filtered_cloud_ptr;
}

MapLoader::PCLPointCloudTypePtr MapLoader::transformPointCloud(
  const PCLPointCloudTypePtr & cloud_ptr) const
{
  if (!cloud_ptr) {
    RCLCPP_ERROR(this->get_logger(), "Input cloud for transform is null");
    return std::make_shared<PCLPointCloudType>();
  }

  // Standard rigid transform: p' = R * p + t
  const Eigen::AngleAxisf roll(static_cast<float>(rotation_roll_), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(static_cast<float>(rotation_pitch_), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw(static_cast<float>(rotation_yaw_), Eigen::Vector3f::UnitZ());

  // Rotation for ROS（yaw->pitch->roll）: R = Rz * Ry * Rx
  const Eigen::Matrix3f R = (yaw * pitch * roll).toRotationMatrix();

  Eigen::Affine3f T = Eigen::Affine3f::Identity();
  T.linear() = R;
  T.translation() = Eigen::Vector3f(
    static_cast<float>(translation_x_), static_cast<float>(translation_y_),
    static_cast<float>(translation_z_));

  auto out = std::make_shared<PCLPointCloudType>();
  pcl::transformPointCloud(*cloud_ptr, *out, T);

  out->header = cloud_ptr->header;
  out->is_dense = cloud_ptr->is_dense;
  return out;
}

}  // namespace terrauto_map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(terrauto_map_loader::MapLoader)
