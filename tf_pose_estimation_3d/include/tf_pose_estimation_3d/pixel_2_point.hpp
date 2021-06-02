// Copyright 2021 Intelligent Robotics Lab
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

/* Author: Jonatan Gines Clavero jonatan.gines@urjc.es */

#ifndef TF_POSE_ESTIMATION_3D__PIXEL2POINT_HPP_
#define TF_POSE_ESTIMATION_3D__PIXEL2POINT_HPP_

#include <string>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include "gb_perception_utils/Perceptor3D.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf_pose_estimation_msgs/msg/frame.hpp"

using namespace tf_pose_estimation_msgs::msg;

namespace tf_pose_estimation_3d
{
class Pixel2Point
{
public:
  Pixel2Point(rclcpp::Node::SharedPtr node);
  void step();

private:
  
  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const Frame::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<Frame>::SharedPtr pose_sub_;
  rclcpp::Publisher<Frame>::SharedPtr pose_3d_pub_, pose_3d_test_pub_;
  std::shared_ptr<Frame> pose_frame_;
  std::shared_ptr<gb_perception_utils::Perceptor3D> perceptor3d_;
};

}  // namespace tf_pose_estimation_3d

#endif  // TF_POSE_ESTIMATION_3D__PIXEL2POINT_HPP_
