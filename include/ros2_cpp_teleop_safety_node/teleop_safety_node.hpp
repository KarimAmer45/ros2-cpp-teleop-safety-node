// Copyright 2026 Karim Amer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <limits>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace ros2_cpp_teleop_safety_node
{

class TeleopSafetyNode final : public rclcpp::Node
{
public:
  explicit TeleopSafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct SafetyParameters
  {
    std::string input_topic{"/cmd_vel_raw"};
    std::string output_topic{"/cmd_vel"};
    std::string scan_topic{"/scan"};
    std::string estop_topic{"/e_stop"};
    std::string speed_limit_topic{"/speed_limit"};
    double command_timeout{0.35};
    double timer_period{0.05};
    double max_linear_speed{0.6};
    double max_lateral_speed{0.25};
    double max_angular_speed{1.4};
    double max_linear_accel{1.0};
    double max_angular_accel{2.0};
    bool allow_lateral_motion{false};
    bool publish_zero_on_timeout{true};
    bool obstacle_stop_enabled{true};
    double min_obstacle_distance{0.45};
    double front_arc_degrees{70.0};
  };

  void loadParameters();
  void handleCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
  void handleScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void handleEstop(const std_msgs::msg::Bool::SharedPtr msg);
  void handleSpeedLimit(const std_msgs::msg::Float64::SharedPtr msg);
  void handleWatchdog();

  geometry_msgs::msg::Twist makeSafeCommand(
    const geometry_msgs::msg::Twist & raw_command,
    const rclcpp::Time & now);
  geometry_msgs::msg::Twist zeroCommand() const;
  void publishCommand(const geometry_msgs::msg::Twist & command, const rclcpp::Time & now);
  void publishZeroNow();
  bool commandTimedOut(const rclcpp::Time & now) const;
  bool obstacleInStopZone() const;

  static double clamp(double value, double min_value, double max_value);
  static double finiteOrZero(double value);
  static double rateLimit(double target, double current, double max_delta);
  static double degreesToRadians(double degrees);

  SafetyParameters params_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_limit_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  geometry_msgs::msg::Twist last_safe_command_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_publish_time_;
  bool have_command_{false};
  bool have_published_{false};
  bool estop_active_{false};
  double speed_limit_scale_{1.0};
  double nearest_front_obstacle_m_{std::numeric_limits<double>::infinity()};
};

}  // namespace ros2_cpp_teleop_safety_node
