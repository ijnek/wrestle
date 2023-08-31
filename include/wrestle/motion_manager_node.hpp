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

#include <memory>

#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

namespace wrestle
{

class MotionManagerNode : public rclcpp::Node
{
public:
  explicit MotionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_getup_front_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_start_getup_back_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  void imuCallback(const sensor_msgs::msg::Imu & msg);
};


}  // namespace wrestle
