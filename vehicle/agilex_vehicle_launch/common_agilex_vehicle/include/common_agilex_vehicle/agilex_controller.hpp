// Copyright 2026 Apollo
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

#ifndef COMMON_AGILEX_VEHICLE__AGILEX_CONTROLLER_HPP_
#define COMMON_AGILEX_VEHICLE__AGILEX_CONTROLLER_HPP_

#include <string>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace common_agilex_vehicle
{

class AgilexJoyController : public rclcpp::Node
{
public:
  explicit AgilexJoyController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  bool has_required_inputs(const sensor_msgs::msg::Joy & msg) const;
  bool button_pressed(const sensor_msgs::msg::Joy & msg, int index) const;
  void autoware_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void watchdog_callback();
  void publish_control_mode();
  void publish_stop();

  std::string joy_topic_;
  std::string control_command_topic_;
  std::string cmd_vel_topic_;
  std::string control_mode_topic_;
  double min_speed_;
  double max_speed_;
  double wheel_base_;
  double max_steering_angle_;
  double min_angular_speed_;
  double max_angular_speed_;
  double input_gain_;
  double deadzone_;
  double speed_limit_step_;
  double speed_limit_factor_;
  double joy_timeout_;
  double control_timeout_;
  int deadman_button_;
  int speed_up_button_;
  int speed_down_button_;
  int speed_axis_;
  int steering_axis_;

  bool previous_speed_up_pressed_{false};
  bool previous_speed_down_pressed_{false};
  bool manual_active_{false};
  bool joy_received_{false};
  bool autonomous_command_received_{false};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_autonomous_command_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr autoware_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace common_agilex_vehicle

#endif  // COMMON_AGILEX_VEHICLE__AGILEX_CONTROLLER_HPP_
