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

#ifndef TERRAUTO_MAP_LOADER__MAP_LOADER_COMPONENT_HPP_
#define TERRAUTO_MAP_LOADER__MAP_LOADER_COMPONENT_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>

namespace terrauto_map_loader
{

class MapLoader : public rclcpp::Node
{
public:
  using PCLPointType = pcl::PointXYZ;
  using PCLPointCloudType = pcl::PointCloud<PCLPointType>;
  using PCLPointCloudTypePtr = std::shared_ptr<PCLPointCloudType>;

  explicit MapLoader(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapLoader() = default;

private:
  void timerCallback();
  PCLPointCloudTypePtr voxelGridFilter(const PCLPointCloudTypePtr & cloud_ptr) const;
  PCLPointCloudTypePtr transformPointCloud(const PCLPointCloudTypePtr & cloud_ptr) const;

  // Publisher and Timer
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  bool use_voxel_grid_filter_;
  double voxel_leaf_x_;
  double voxel_leaf_y_;
  double voxel_leaf_z_;
  double translation_x_;
  double translation_y_;
  double translation_z_;
  double rotation_roll_;
  double rotation_pitch_;
  double rotation_yaw_;
  std::string global_frame_id_;
  std::string map_file_;
  std::string file_type_;
};

}  // namespace terrauto_map_loader

#endif  // TERRAUTO_MAP_LOADER__MAP_LOADER_COMPONENT_HPP_
