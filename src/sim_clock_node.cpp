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

#include "wrestle/sim_clock_node.hpp"

#define DT 0.02

using namespace std::chrono_literals;

// WebotsLolaController abuses the battery's temperature field to transmit the cycle time from
// webots. (Described in https://github.com/Bembelbots/WebotsLoLaController)
//
// This node uses that cycle count, multiplies it by dt, and publishes it on the /clock topic.

namespace wrestle
{

SimClockNode::SimClockNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"sim_clock_node", options}
{
  // Publishers
  pub_clock_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // Subscriptions
  sub_battery_ = create_subscription<nao_lola_sensor_msgs::msg::Battery>(
    "battery", 10, std::bind(&SimClockNode::batteryCallback, this, std::placeholders::_1));
}

void SimClockNode::batteryCallback(const nao_lola_sensor_msgs::msg::Battery & msg)
{
  int64_t t_ns = msg.temperature * (DT * 1e9);
  rclcpp::Time time(t_ns, RCL_ROS_TIME);
  rosgraph_msgs::msg::Clock clock;
  clock.clock = time;
  pub_clock_->publish(clock);
}

}  // namespace wrestle

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wrestle::SimClockNode)
