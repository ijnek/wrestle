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

namespace wrestle
{

class MotionManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"motion_manager_node", options}
{
  pub_start_getup_front_ = create_publisher<std_msgs::msg::Bool>("start_getup_front", 1);
  pub_start_getup_back_ = create_publisher<std_msgs::msg::Bool>("start_getup_back", 1);
}

}  // namespace wrestle
