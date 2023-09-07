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

#include "nao_lola_sensor_msgs/msg/battery.hpp"
#include "rclcpp/node.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace wrestle
{

class SimClockNode : public rclcpp::Node
{
public:
  explicit SimClockNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;

  rclcpp::Subscription<nao_lola_sensor_msgs::msg::Battery>::SharedPtr sub_battery_;

  void batteryCallback(const nao_lola_sensor_msgs::msg::Battery & msg);
};


}  // namespace wrestle
