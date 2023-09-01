// Copyright 2023 Kenji Brameld
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

#include "wrestle/motion_manager_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

namespace wrestle
{

// Forward declarations
float calculateRobotPitch(const sensor_msgs::msg::Imu & msg);

MotionManagerNode::MotionManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"motion_manager_node", options}
{
  // Publishers
  pub_start_getup_front_ = create_publisher<std_msgs::msg::Bool>("start_getup_front", 1);
  pub_start_getup_back_ = create_publisher<std_msgs::msg::Bool>("start_getup_back", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("twist", 1);

  // Subscriptions
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&MotionManagerNode::imuCallback, this, std::placeholders::_1));

  // Timer
  timer_ = create_wall_timer(10ms, std::bind(&MotionManagerNode::timerCallback, this));
}

void MotionManagerNode::imuCallback(const sensor_msgs::msg::Imu & msg)
{
  pitch_ = calculateRobotPitch(msg);
}

void MotionManagerNode::timerCallback()
{
  if (pitch_ > 1.0) {
    std_msgs::msg::Bool start_getup_front;
    start_getup_front.data = true;
    pub_start_getup_front_->publish(start_getup_front);
    return;
  }

  if (pitch_ < -1.0) {
    std_msgs::msg::Bool start_getup_back;
    start_getup_back.data = true;
    pub_start_getup_back_->publish(start_getup_back);
    return;
  }

  // Walk forward
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.1;
  pub_twist_->publish(twist);
}

float calculateRobotPitch(const sensor_msgs::msg::Imu & msg)
{
  tf2::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return pitch;
}

}  // namespace wrestle

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wrestle::MotionManagerNode)
