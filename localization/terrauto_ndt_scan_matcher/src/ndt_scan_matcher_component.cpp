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

#include "terrauto_ndt_scan_matcher/ndt_scan_matcher_component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace terrauto_ndt_scan_matcher
{

template <typename T>
void reallocateCudaManaged(
  T *& ptr, std::size_t new_num_elements, std::size_t & allocated_elements, cudaStream_t stream)
{
  if (allocated_elements < new_num_elements) {
    if (ptr) {
      cudaFree(ptr);
      ptr = nullptr;
    }
    cudaMallocManaged(&ptr, sizeof(T) * new_num_elements, cudaMemAttachHost);
    cudaStreamAttachMemAsync(stream, ptr);
    allocated_elements = new_num_elements;
  }
}

NdtScanMatcher::NdtScanMatcher(const rclcpp::NodeOptions & options)
: Node("ndt_scan_matcher_node", options),
  has_initial_pose_(false),
  has_map_(false),
  has_T_lidar_to_base_(false),
  T_lidar_to_base_(Eigen::Matrix4f::Identity()),
  map_cloud_ptr_(nullptr),
  stream_(nullptr),
  buf_source_(nullptr),
  buf_target_(nullptr),
  buf_guess_T_(nullptr),
  buf_result_T_(nullptr),
  allocated_source_elements_(0),
  allocated_target_elements_(0)
{
  using std::placeholders::_1;

  // Parameters
  bool set_initial_pose;
  double initial_pose_x;
  double initial_pose_y;
  double initial_pose_z;
  double initial_pose_qx;
  double initial_pose_qy;
  double initial_pose_qz;
  double initial_pose_qw;

  this->declare_parameter<bool>("publish_tf", false);
  this->declare_parameter<bool>("set_initial_pose", true);
  this->declare_parameter<int>("min_map_points", 300);
  this->declare_parameter<int>("min_sensor_points", 100);
  this->declare_parameter<int>("ndt.max_iterations", 35);
  this->declare_parameter<double>("ndt.resolution", 1.0);
  this->declare_parameter<double>("ndt.step_size", 0.1);
  this->declare_parameter<double>("ndt.transform_epsilon", 0.01);
  this->declare_parameter<double>("score.threshold", 2.0);
  this->declare_parameter<double>("score.cutoff_ratio", 0.1);
  this->declare_parameter<double>("initial_pose.x", 0.0);
  this->declare_parameter<double>("initial_pose.y", 0.0);
  this->declare_parameter<double>("initial_pose.z", 0.0);
  this->declare_parameter<double>("initial_pose.qx", 0.0);
  this->declare_parameter<double>("initial_pose.qy", 0.0);
  this->declare_parameter<double>("initial_pose.qz", 0.0);
  this->declare_parameter<double>("initial_pose.qw", 1.0);
  this->declare_parameter<double>("cov.base.xy", 0.25);
  this->declare_parameter<double>("cov.base.z", 0.25);
  this->declare_parameter<double>("cov.base.rpy", 0.0075);
  this->declare_parameter<double>("cov.scale_max", 100.0);
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<std::string>("base_frame_id", "base_link");

  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("set_initial_pose", set_initial_pose);
  this->get_parameter("min_map_points", min_map_points_);
  this->get_parameter("min_sensor_points", min_sensor_points_);
  this->get_parameter("ndt.max_iterations", ndt_max_iterations_);
  this->get_parameter("ndt.resolution", ndt_resolution_);
  this->get_parameter("ndt.step_size", ndt_step_size_);
  this->get_parameter("ndt.transform_epsilon", ndt_transform_epsilon_);
  this->get_parameter("score.threshold", score_threshold_);
  this->get_parameter("score.cutoff_ratio", score_cutoff_ratio_);
  this->get_parameter("initial_pose.x", initial_pose_x);
  this->get_parameter("initial_pose.y", initial_pose_y);
  this->get_parameter("initial_pose.z", initial_pose_z);
  this->get_parameter("initial_pose.qx", initial_pose_qx);
  this->get_parameter("initial_pose.qy", initial_pose_qy);
  this->get_parameter("initial_pose.qz", initial_pose_qz);
  this->get_parameter("initial_pose.qw", initial_pose_qw);
  this->get_parameter("cov.base.xy", cov_xy_base_);
  this->get_parameter("cov.base.z", cov_z_base_);
  this->get_parameter("cov.base.rpy", cov_rpy_base_);
  this->get_parameter("cov.scale_max", cov_scale_max_);
  this->get_parameter("global_frame_id", global_frame_id_);
  this->get_parameter("base_frame_id", base_frame_id_);

  // CUDA init
  cudaStreamCreate(&stream_);

  cudaMallocManaged(&buf_guess_T_, sizeof(float) * 16, cudaMemAttachHost);
  cudaStreamAttachMemAsync(stream_, buf_guess_T_);

  cudaMallocManaged(&buf_result_T_, sizeof(float) * 16, cudaMemAttachHost);
  cudaStreamAttachMemAsync(stream_, buf_result_T_);
  cudaMemsetAsync(buf_result_T_, 0, sizeof(float) * 16, stream_);

  kd_tree_ = std::make_shared<pcl::search::KdTree<PCLPointType>>();

  // TF
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Publisher
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ndt_pose", rclcpp::QoS(10).reliable().durability_volatile());

  // Callback Group
  rclcpp::CallbackGroup::SharedPtr pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pose_sub_opt = rclcpp::SubscriptionOptions();
  pose_sub_opt.callback_group = pose_callback_group;

  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  // Subscription
  sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(5).reliable().durability_volatile(),
    std::bind(&NdtScanMatcher::initialPoseCallback, this, _1), pose_sub_opt);
  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
    std::bind(&NdtScanMatcher::poseCallback, this, _1), pose_sub_opt);
  sub_map_ = this->create_subscription<PointCloudAdaptedType>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&NdtScanMatcher::mapCallback, this, _1), sensor_sub_opt);
  sub_points_ = this->create_subscription<PointCloudAdaptedType>(
    "points", rclcpp::SensorDataQoS(), std::bind(&NdtScanMatcher::pointsCallback, this, _1),
    sensor_sub_opt);

  if (set_initial_pose) {
    auto initial_pose_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    initial_pose_ptr->header.stamp = this->get_clock()->now();
    initial_pose_ptr->header.frame_id = global_frame_id_;
    initial_pose_ptr->pose.pose.position.x = initial_pose_x;
    initial_pose_ptr->pose.pose.position.y = initial_pose_y;
    initial_pose_ptr->pose.pose.position.z = initial_pose_z;
    initial_pose_ptr->pose.pose.orientation.x = initial_pose_qx;
    initial_pose_ptr->pose.pose.orientation.y = initial_pose_qy;
    initial_pose_ptr->pose.pose.orientation.z = initial_pose_qz;
    initial_pose_ptr->pose.pose.orientation.w = initial_pose_qw;

    initialPoseCallback(initial_pose_ptr);
    publishPose();
  }
}

NdtScanMatcher::~NdtScanMatcher()
{
  if (buf_source_) cudaFree(buf_source_);
  if (buf_target_) cudaFree(buf_target_);
  if (buf_guess_T_) cudaFree(buf_guess_T_);
  if (buf_result_T_) cudaFree(buf_result_T_);
  if (stream_) cudaStreamDestroy(stream_);
}

void NdtScanMatcher::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  pose_current_ = *msg;

  setPoseCovarianceByFitnessScore(pose_current_, 0.0);

  has_initial_pose_ = true;
}

void NdtScanMatcher::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  pose_current_ = *msg;
}

void NdtScanMatcher::mapCallback(const PCLPointCloudTypePtr & cloud_ptr)
{
  if (!cloud_ptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cloud_ptr is null. Skip.");
    return;
  }

  if (static_cast<int>(cloud_ptr->size()) < min_map_points_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Too few map points: %zu (<%d). Skip.",
      cloud_ptr->size(), min_map_points_);
    return;
  }

  PCLPointCloudTypePtr cloud_ptr_in_global = cloud_ptr;

  // map点群は以降の scan matching / fitness 計算の基準になるため、global_frame_id_ に揃える
  if (cloud_ptr->header.frame_id != global_frame_id_) {
    geometry_msgs::msg::TransformStamped tf_src_to_global;
    try {
      tf_src_to_global = tf_buffer_->lookupTransform(
        global_frame_id_, cloud_ptr->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Could not transform map points: %s",
        ex.what());
      return;
    }

    const Eigen::Matrix4f T_src_to_global =
      tf2::transformToEigen(tf_src_to_global.transform).matrix().cast<float>();

    cloud_ptr_in_global = std::make_shared<PCLPointCloudType>();
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr_in_global, T_src_to_global);

    cloud_ptr_in_global->header = cloud_ptr->header;
    cloud_ptr_in_global->header.frame_id = global_frame_id_;
  }

  std::lock_guard<std::mutex> lock(sensor_mutex_);
  map_cloud_ptr_ = cloud_ptr_in_global;
  kd_tree_->setInputCloud(map_cloud_ptr_);
  has_map_ = true;
}

void NdtScanMatcher::pointsCallback(const PCLPointCloudTypePtr & cloud_ptr)
{
  if (!cloud_ptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cloud_ptr is null. Skip.");
    return;
  }

  if (static_cast<int>(cloud_ptr->size()) < min_sensor_points_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Too few sensor points: %zu (<%d). Skip.",
      cloud_ptr->size(), min_sensor_points_);
    return;
  }

  // タイムスタンプ変換（PCL → ROS）
  rclcpp::Time ros_msg_time;
  pcl_conversions::fromPCL(cloud_ptr->header.stamp, ros_msg_time);

  // LiDAR→base の変換を初回に取得してキャッシュ
  if (!has_T_lidar_to_base_) {
    geometry_msgs::msg::TransformStamped tf_lidar_to_base;
    try {
      tf_lidar_to_base =
        tf_buffer_->lookupTransform(base_frame_id_, cloud_ptr->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Could not transform sensor points: %s",
        ex.what());
      return;
    }
    T_lidar_to_base_ = tf2::transformToEigen(tf_lidar_to_base.transform).matrix().cast<float>();
    has_T_lidar_to_base_ = true;
    RCLCPP_INFO(this->get_logger(), "Cached lidar->base transform.");
  }

  auto sensor_cloud_ptr_in_base = std::make_shared<PCLPointCloudType>();
  pcl::transformPointCloud(*cloud_ptr, *sensor_cloud_ptr_in_base, T_lidar_to_base_);

  PCLPointCloudTypePtr map_cloud_ptr;
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (!has_map_) {
      return;
    }
    map_cloud_ptr = map_cloud_ptr_;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_init;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!has_initial_pose_) {
      return;
    }
    pose_init = pose_current_;
  }

  const std::size_t num_source_points = sensor_cloud_ptr_in_base->size();
  const std::size_t num_target_points = map_cloud_ptr->size();

  float * source_data = reinterpret_cast<float *>(sensor_cloud_ptr_in_base->points.data());
  float * target_data = reinterpret_cast<float *>(map_cloud_ptr->points.data());

  reallocateCudaManaged(buf_source_, 4 * num_source_points, allocated_source_elements_, stream_);
  reallocateCudaManaged(buf_target_, 4 * num_target_points, allocated_target_elements_, stream_);

  cudaMemcpyAsync(
    buf_source_, source_data, sizeof(float) * 4 * num_source_points, cudaMemcpyHostToDevice,
    stream_);
  cudaMemcpyAsync(
    buf_target_, target_data, sizeof(float) * 4 * num_target_points, cudaMemcpyHostToDevice,
    stream_);

  // 初期値（推定開始姿勢）をセット
  const Eigen::Matrix4f T_init = poseToMatrix4f(pose_init.pose.pose);
  cudaMemcpyAsync(buf_guess_T_, T_init.data(), sizeof(float) * 16, cudaMemcpyHostToDevice, stream_);
  cudaMemsetAsync(buf_result_T_, 0, sizeof(float) * 16, stream_);

  cudaStreamSynchronize(stream_);

  cudaNDT cuda_ndt(num_source_points, num_target_points, stream_);
  cuda_ndt.setInputSource(reinterpret_cast<void *>(buf_source_));
  cuda_ndt.setInputTarget(reinterpret_cast<void *>(buf_target_));
  cuda_ndt.setResolution(ndt_resolution_);
  cuda_ndt.setMaximumIterations(ndt_max_iterations_);
  cuda_ndt.setTransformationEpsilon(ndt_transform_epsilon_);
  cuda_ndt.setStepSize(ndt_step_size_);

  auto start_time = std::chrono::steady_clock::now();
  cuda_ndt.ndt(
    buf_source_, num_source_points, buf_target_, num_target_points, buf_guess_T_, buf_result_T_,
    stream_);
  cudaStreamSynchronize(stream_);
  auto end_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_time - start_time;
  RCLCPP_DEBUG(this->get_logger(), "[cudaNDT] Align Time: %lf [s]", elapsed_seconds.count());

  Eigen::Matrix4f T_final = Eigen::Matrix4f::Identity();
  memcpy(T_final.data(), buf_result_T_, sizeof(float) * 16);

  // Fitness scoreの計算
  // fitness が悪い場合は pose を更新せず前回値を維持（破綻で飛ぶのを抑制）
  const double fitness_score = calculateFitnessScore(sensor_cloud_ptr_in_base, T_final);
  const bool is_good = (fitness_score < score_threshold_);

  const geometry_msgs::msg::Pose pose_latest = matrix4fToPose(T_final);

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_current_.header.stamp = ros_msg_time;
    pose_current_.header.frame_id = global_frame_id_;
    if (is_good) {
      pose_current_.pose.pose = pose_latest;
    }
    setPoseCovarianceByFitnessScore(pose_current_, fitness_score);
  }

  publishPose();
}

void NdtScanMatcher::publishPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_copy;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_copy = pose_current_;
  }

  pub_pose_->publish(pose_copy);

  if (!publish_tf_) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_out;
  tf_out.header.stamp = pose_copy.header.stamp;
  tf_out.header.frame_id = global_frame_id_;
  tf_out.child_frame_id = base_frame_id_;
  tf_out.transform.translation.x = pose_copy.pose.pose.position.x;
  tf_out.transform.translation.y = pose_copy.pose.pose.position.y;
  tf_out.transform.translation.z = pose_copy.pose.pose.position.z;
  tf_out.transform.rotation = pose_copy.pose.pose.orientation;

  tf_broadcaster_->sendTransform(tf_out);
}

/*
 *  fitness scoreに応じて covariance を計算する関数
 *
 *  x = clamp((fitness_score / score_threshold), 0, 1)
 *  scale = 1 + (scale_max - 1) * x^2
 *
 *  分散（sigma^2） = 分散の基準値（sigma^2_base） * scale
 */
void NdtScanMatcher::setPoseCovarianceByFitnessScore(
  geometry_msgs::msg::PoseWithCovarianceStamped & pose, double fitness_score) const
{
  double scale = cov_scale_max_;

  if (std::isfinite(fitness_score)) {
    const double x = std::clamp(fitness_score / score_threshold_, 0.0, 1.0);
    scale = 1.0 + (cov_scale_max_ - 1.0) * (x * x);
  }

  std::fill(std::begin(pose.pose.covariance), std::end(pose.pose.covariance), 0.0);

  pose.pose.covariance[0] = cov_xy_base_ * scale;    // x
  pose.pose.covariance[7] = cov_xy_base_ * scale;    // y
  pose.pose.covariance[14] = cov_z_base_ * scale;    // z
  pose.pose.covariance[21] = cov_rpy_base_ * scale;  // roll
  pose.pose.covariance[28] = cov_rpy_base_ * scale;  // pitch
  pose.pose.covariance[35] = cov_rpy_base_ * scale;  // yaw
}

double NdtScanMatcher::calculateFitnessScore(
  const PCLPointCloudTypePtr & sensor_cloud_ptr, const Eigen::Matrix4f & T_sensor_to_map)
{
  // 1) 推定結果の変換を点群に適用し、map座標系へ変換する
  PCLPointCloudType sensor_cloud_in_map;
  pcl::transformPointCloud(*sensor_cloud_ptr, sensor_cloud_in_map, T_sensor_to_map);

  // 2) map点群に対する最近傍距離を計算する（KdTree）
  std::vector<int> nn_indices(1);
  std::vector<float> nn_sq_dists(1);
  std::vector<float> nn_dists_all;
  for (const auto & pt : sensor_cloud_in_map.points) {
    const int found = kd_tree_->nearestKSearch(pt, 1, nn_indices, nn_sq_dists);
    if (found > 0) {
      nn_dists_all.push_back(std::sqrt(std::max(0.0f, nn_sq_dists[0])));
    }
  }

  if (nn_dists_all.empty()) {
    return std::numeric_limits<double>::max();
  }

  // 3) 距離を昇順ソートする
  std::sort(nn_dists_all.begin(), nn_dists_all.end());

  // 4) 外れ値（対応が取れない点、動体など）の影響を減らすため、
  //    距離が大きい上位 score_cutoff_ratio_ を除外する
  const std::size_t num_used =
    static_cast<std::size_t>(nn_dists_all.size() * (1.0 - score_cutoff_ratio_));

  if (num_used == 0) {
    return std::numeric_limits<double>::max();
  }

  double sum = 0.0;
  for (std::size_t i = 0; i < num_used; ++i) {
    sum += static_cast<double>(nn_dists_all[i]);
  }

  // 5) トリミング平均（小さいほど整合が良い）を返す
  return sum / static_cast<double>(num_used);
}

}  // namespace terrauto_ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(terrauto_ndt_scan_matcher::NdtScanMatcher)
