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

#ifndef TERRAUTO_NDT_SCAN_MATCHER__UTILS_HPP_
#define TERRAUTO_NDT_SCAN_MATCHER__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace terrauto_ndt_scan_matcher
{

Eigen::Affine3d poseToAffine3d(const geometry_msgs::msg::Pose & ros_pose);
Eigen::Matrix4f poseToMatrix4f(const geometry_msgs::msg::Pose & ros_pose);

geometry_msgs::msg::Point matrix4fToPoint(const Eigen::Matrix4f & eigen_pose_mat);
geometry_msgs::msg::Pose matrix4fToPose(const Eigen::Matrix4f & eigen_pose_mat);
geometry_msgs::msg::Quaternion matrix3dToQuat(const Eigen::Matrix3d & eigen_rot_mat);

}  // namespace terrauto_ndt_scan_matcher

#endif  // TERRAUTO_NDT_SCAN_MATCHER__UTILS_HPP_
