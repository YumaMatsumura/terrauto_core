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

#include "terrauto_ndt_scan_matcher/utils.hpp"

namespace terrauto_ndt_scan_matcher
{

Eigen::Affine3d poseToAffine3d(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose;
  tf2::fromMsg(ros_pose, eigen_pose);
  return eigen_pose;
}

Eigen::Matrix4f poseToMatrix4f(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose_affine = poseToAffine3d(ros_pose);
  Eigen::Matrix4d eigen_pose_matrix_d = eigen_pose_affine.matrix();
  Eigen::Matrix4f eigen_pose_matrix = eigen_pose_matrix_d.cast<float>();
  return eigen_pose_matrix;
}

geometry_msgs::msg::Point matrix4fToPoint(const Eigen::Matrix4f & eigen_pose_mat)
{
  geometry_msgs::msg::Point ros_point;
  ros_point.x = static_cast<double>(eigen_pose_mat(0, 3));
  ros_point.y = static_cast<double>(eigen_pose_mat(1, 3));
  ros_point.z = static_cast<double>(eigen_pose_mat(2, 3));
  return ros_point;
}

geometry_msgs::msg::Pose matrix4fToPose(const Eigen::Matrix4f & eigen_pose_mat)
{
  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position = matrix4fToPoint(eigen_pose_mat);

  Eigen::Matrix3d rot_mat = eigen_pose_mat.block<3, 3>(0, 0).cast<double>();
  ros_pose.orientation = matrix3dToQuat(rot_mat);
  return ros_pose;
}

geometry_msgs::msg::Quaternion matrix3dToQuat(const Eigen::Matrix3d & eigen_rot_mat)
{
  Eigen::Quaterniond eigen_quat(eigen_rot_mat);
  eigen_quat.normalize();
  geometry_msgs::msg::Quaternion ros_quat = tf2::toMsg(eigen_quat);
  return ros_quat;
}

}  // namespace terrauto_ndt_scan_matcher
