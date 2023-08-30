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
#include "nao_lola_sensor_msgs/msg/angle.hpp"
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

  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Angle>::SharedPtr sub_angle_;

  void angleCallback(const nao_lola_sensor_msgs::msg::Angle & msg);
};


}  // namespace wrestle
