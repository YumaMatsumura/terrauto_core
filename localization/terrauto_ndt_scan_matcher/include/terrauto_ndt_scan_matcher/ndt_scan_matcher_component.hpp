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

#ifndef TERRAUTO_NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_COMPONENT_HPP_
#define TERRAUTO_NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_COMPONENT_HPP_

#include "cudaNDT.h"
#include "cuda_runtime.h"
#include "terrauto_ndt_scan_matcher/utils.hpp"
#include "terrauto_type_adapter/pcl_type_adapter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace terrauto_ndt_scan_matcher
{

template <typename T>
void reallocateCudaManaged(
  T *& ptr, std::size_t new_num_elements, std::size_t & allocated_elements, cudaStream_t stream);

class NdtScanMatcher : public rclcpp::Node
{
public:
  using PCLPointType = pcl::PointXYZ;
  using PCLPointCloudType = pcl::PointCloud<PCLPointType>;
  using PCLPointCloudTypePtr = std::shared_ptr<PCLPointCloudType>;
  using PointCloudAdaptedType =
    rclcpp::TypeAdapter<PCLPointCloudTypePtr, sensor_msgs::msg::PointCloud2>;

  explicit NdtScanMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NdtScanMatcher();

private:
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapCallback(const PCLPointCloudTypePtr & cloud_ptr);
  void pointsCallback(const PCLPointCloudTypePtr & cloud_ptr);
  void publishPose();
  void setPoseCovarianceByFitnessScore(
    geometry_msgs::msg::PoseWithCovarianceStamped & pose, double fitness_score) const;
  double calculateFitnessScore(
    const PCLPointCloudTypePtr & sensor_cloud_ptr, const Eigen::Matrix4f & T_sensor_to_map);

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<PointCloudAdaptedType>::SharedPtr sub_map_;
  rclcpp::Subscription<PointCloudAdaptedType>::SharedPtr sub_points_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  bool has_initial_pose_;
  bool has_map_;
  bool has_T_lidar_to_base_;

  Eigen::Matrix4f T_lidar_to_base_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_current_;
  PCLPointCloudTypePtr map_cloud_ptr_;
  pcl::search::KdTree<PCLPointType>::Ptr kd_tree_;

  mutable std::mutex pose_mutex_;
  mutable std::mutex sensor_mutex_;

  cudaStream_t stream_;
  float * buf_source_;
  float * buf_target_;
  float * buf_guess_T_;
  float * buf_result_T_;
  std::size_t allocated_source_elements_;
  std::size_t allocated_target_elements_;

  bool publish_tf_;
  int min_map_points_;
  int min_sensor_points_;
  int ndt_max_iterations_;
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_transform_epsilon_;
  double score_threshold_;
  double score_cutoff_ratio_;
  double cov_xy_base_;
  double cov_z_base_;
  double cov_rpy_base_;
  double cov_scale_max_;
  std::string global_frame_id_;
  std::string base_frame_id_;
};

}  // namespace terrauto_ndt_scan_matcher

#endif  // TERRAUTO_NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_COMPONENT_HPP_
