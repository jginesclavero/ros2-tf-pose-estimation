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

#include <stdlib.h>
#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <algorithm>

#include "tf_pose_estimation_3d/pixel_2_point.hpp"

using std::placeholders::_1;

namespace tf_pose_estimation_3d
{

Pixel2Point::Pixel2Point(rclcpp::Node::SharedPtr node) : node_(node)
{
  //point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
  //  "/xtion/depth_registered/points", 1, std::bind(&Pixel2Point::pointCloudCb, this, std::placeholders::_1));
  pose_sub_ = node_->create_subscription<Frame>(
    "/pose", 1, std::bind(&Pixel2Point::poseCallback, this, std::placeholders::_1));
  pose_3d_pub_ = node_->create_publisher<Frame>("/pose_3d", 1);
  pose_3d_test_pub_ = node_->create_publisher<Frame>("/pose_3d_test", 1);

  pose_frame_ = std::make_shared<Frame>();
  perceptor3d_ = std::make_shared<gb_perception_utils::Perceptor3D> (node_);
}

void 
Pixel2Point::pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg<pcl::PointXYZ>(*msg, *cloud);
  for (auto &person : pose_frame_->persons)
  {
    for (auto &part : person.body_parts)
    {
      pcl::PointXYZ p = cloud->at(part.pixel.x, part.pixel.y);
      part.point.x = p.x;
      part.point.y = p.y;
      part.point.z = p.z;
    }
  }
  pose_3d_pub_->publish(*pose_frame_);
}

void 
Pixel2Point::poseCallback(const Frame::SharedPtr msg)
{
  pose_frame_ = msg;
}

void 
Pixel2Point::step()
{
  rclcpp::Time now = node_->now();
  for (auto &person : pose_frame_->persons)
  {
    for (auto &part : person.body_parts)  
    {
      //pcl::PointXYZ p = cloud->at(part.pixel.x, part.pixel.y);
      auto point3d = perceptor3d_->get_3d_from_pixel(
        part.pixel.x, 
        part.pixel.y, 
        pose_frame_->header.stamp, 
        pose_frame_->header.frame_id);
      if (point3d.has_value())
      {
        part.point.x = point3d.value().x();
        part.point.y = point3d.value().y();
        part.point.z = point3d.value().z();
      }
    }
  }
  pose_3d_pub_->publish(*pose_frame_);
}

}  // namespace tf_pose_estimation_3d

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pixel_2_point");
  auto pixel2point = std::make_shared<tf_pose_estimation_3d::Pixel2Point>(node);
  rclcpp::Rate rate(30);
  while (rclcpp::ok()) 
  {
    pixel2point->step();
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  
  exit(EXIT_SUCCESS);
}
