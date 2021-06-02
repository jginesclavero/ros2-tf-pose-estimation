
// Copyright 2020 Intelligent Robotics Lab
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

/* Author: jginesclavero jonatan.gines@urjc.es */

/* Mantainer: jginesclavero jonatan.gines@urjc.es */
#ifndef TF_POSE_ESTIMATION_VIZ__PUBLISH_MARKERS_H
#define TF_POSE_ESTIMATION_VIZ__PUBLISH_MARKERS_H

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/empty.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf_pose_estimation_msgs/msg/frame.hpp"

using namespace tf_pose_estimation_msgs::msg;

namespace tf_pose_estimation_viz
{
class PublishMarkers : public rclcpp::Node
{
public:

  PublishMarkers(const std::string & name);
  void step();

private:
  void poseCallback(const Frame::SharedPtr msg);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<Frame>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  std::string target_frame_;

  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};
};  // namespace tf_pose_estimation_viz

#endif

