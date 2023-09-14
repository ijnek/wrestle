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

#include "rclcpp/executors.hpp"
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
  pub_start_lean_forward_ = create_publisher<std_msgs::msg::Bool>("start_lean_forward", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::Twist>("twist", 1);

  // Services
  srv_walk_change_state_ = create_client<lifecycle_msgs::srv::ChangeState>("walk_change_state");

  // Subscriptions
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&MotionManagerNode::imuCallback, this, std::placeholders::_1));
  sub_sonar_ = create_subscription<nao_lola_sensor_msgs::msg::Sonar>(
    "sonar", 10, std::bind(&MotionManagerNode::sonarCallback, this, std::placeholders::_1));

  // Timer
  timer_ = create_wall_timer(10ms, std::bind(&MotionManagerNode::timerCallback, this));
  timer_use_sonar_readings_ =
    create_wall_timer(4000ms, std::bind(&MotionManagerNode::timerUseSonarReadingsCallback, this));

  start_time_ = now();
}

void MotionManagerNode::imuCallback(const sensor_msgs::msg::Imu & msg)
{
  pitch_ = calculateRobotPitch(msg);
}

void MotionManagerNode::sonarCallback(const nao_lola_sensor_msgs::msg::Sonar & msg)
{
  obstacle_in_front_ = (msg.left < 0.4 || msg.right < 0.4);
}

void MotionManagerNode::timerCallback()
{
  if (pitch_ > 1.0) {
    stopWalk();

    std_msgs::msg::Bool start_getup_front;
    start_getup_front.data = true;
    pub_start_getup_front_->publish(start_getup_front);
    return;
  }

  if (pitch_ < -1.0) {
    stopWalk();

    std_msgs::msg::Bool start_getup_back;
    start_getup_back.data = true;
    pub_start_getup_back_->publish(start_getup_back);
    return;
  }

  if (use_sonar_readings_ && obstacle_in_front_) {
    stopWalk();

    std_msgs::msg::Bool start_lean_forward;
    start_lean_forward.data = true;
    pub_start_lean_forward_->publish(start_lean_forward);
    return;
  }

  // Walk
  auto time_elapsed = now() - start_time_;

  geometry_msgs::msg::Twist target_twist;
  if (time_elapsed < rclcpp::Duration(10s)) {
    // Move forward
    target_twist.linear.x = 0.1;
  } else if (time_elapsed < rclcpp::Duration(25s)) {
    // Move left
    target_twist.linear.y = 0.1;
  } else if (time_elapsed < rclcpp::Duration(55s)) {
    // Move right
    target_twist.linear.y = -0.1;
  } else if (time_elapsed < rclcpp::Duration(85s)) {
    // Move forward
    target_twist.linear.x = 0.1;
  }

  pub_twist_->publish(target_twist);
}

void MotionManagerNode::timerUseSonarReadingsCallback()
{
  use_sonar_readings_ = true;
  timer_use_sonar_readings_->cancel();
}

void MotionManagerNode::stopWalk()
{
  return;  // (TODO): Remove this. This is here since this node lifecycle transition isn't really working yet.

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  auto future_result = srv_walk_change_state_->async_send_request(request);
  try {
    if (rclcpp::spin_until_future_complete(
        get_node_base_interface(),
        future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_DEBUG(get_logger(), "Failed to call service walk_change_state");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Service call failed: " << e.what());
  }
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
