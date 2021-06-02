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

#include "rclcpp/rclcpp.hpp"
#include "tf_pose_estimation_viz/publish_markers.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace tf_pose_estimation_msgs::msg;
using namespace geometry_msgs::msg;

namespace tf_pose_estimation_viz
{
PublishMarkers::PublishMarkers(const std::string & name) : Node(name)
{
  sub_ = create_subscription<Frame>(
      "/pose_3d", 1,
      std::bind(&PublishMarkers::poseCallback, this, std::placeholders::_1));
  //tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/pose_estimation/markers", 1);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PublishMarkers::poseCallback(const Frame::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (auto person : msg->persons)
  {
    for (auto part : person.body_parts)
    {
      TransformStamped bf2oframe;
      visualization_msgs::msg::Marker marker;
      PointStamped part_pt, tf_part_pt;
      try {
        part_pt.point = part.point;
        part_pt.header.frame_id = msg->header.frame_id;
        part_pt.header.stamp = msg->header.stamp;
        tf_buffer_->transform(part_pt, tf_part_pt, "base_footprint");
      } catch (tf2::TransformException & e) {
        RCLCPP_WARN(get_logger(), "%s", e.what());
        return;
      }
      
      if (part_pt.point.x == 0.0 && part_pt.point.y == 0.0 && part_pt.point.z == 0.0)
      {
        continue;
      }

      marker.header.frame_id = "base_footprint";
      marker.header.stamp = msg->header.stamp;
      marker.ns = "pose_estimation";
      marker.id = std::stoi(part.id);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.frame_locked = false;
      marker.pose.position.x = tf_part_pt.point.x;
      marker.pose.position.y = tf_part_pt.point.y;
      marker.pose.position.z = tf_part_pt.point.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.lifetime = rclcpp::Duration(1.0);
      marker.text = part.id;
      marker_array.markers.push_back(marker);
    }
  }
  markers_pub_->publish(marker_array);
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub_markers = 
    std::make_shared<tf_pose_estimation_viz::PublishMarkers>("pub_markers");
  //rclcpp::Rate loop_rate(100ms); 
  //while (rclcpp::ok())
  //{
  //  yolact3d_tf2->step();
  //  rclcpp::spin_some(yolact3d_tf2);
  // loop_rate.sleep();
  //}
  rclcpp::spin(pub_markers);
  rclcpp::shutdown();

  return 0;
}