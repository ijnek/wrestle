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

namespace wrestle
{

MotionManagerNode::MotionManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"motion_manager_node", options}
{
  // Publishers
  pub_start_getup_front_ = create_publisher<std_msgs::msg::Bool>("start_getup_front", 1);
  pub_start_getup_back_ = create_publisher<std_msgs::msg::Bool>("start_getup_back", 1);

  // Subscriptions
  sub_angle_ = create_subscription<nao_lola_sensor_msgs::msg::Angle>(
    "imu", 10, std::bind(&MotionManagerNode::angleCallback, this, std::placeholders::_1));
}

void MotionManagerNode::angleCallback(const nao_lola_sensor_msgs::msg::Angle & msg)
{
  if (msg.x > 0.5) {
    std_msgs::msg::Bool start_getup_front;
    start_getup_front.data = true;
    pub_start_getup_front_->publish(start_getup_front);
  } else if (msg.x < -0.5) {
    std_msgs::msg::Bool start_getup_back;
    start_getup_back.data = true;
    pub_start_getup_back_->publish(start_getup_back);
  }
}

}  // namespace wrestle

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wrestle::MotionManagerNode)
