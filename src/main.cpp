// Copyright 2026 Karim Amer
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "rclcpp/rclcpp.hpp"
#include "ros2_cpp_teleop_safety_node/teleop_safety_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_cpp_teleop_safety_node::TeleopSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
