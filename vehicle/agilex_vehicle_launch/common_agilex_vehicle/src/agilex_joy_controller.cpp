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

#include "common_agilex_vehicle/agilex_joy_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>

namespace common_agilex_vehicle
{

AgilexJoyController::AgilexJoyController(const rclcpp::NodeOptions & options)
: Node("agilex_joy_controller", options)
{
  joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
  control_command_topic_ = declare_parameter<std::string>(
    "control_command_topic", "/control/command/control_cmd");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  control_mode_topic_ = declare_parameter<std::string>(
    "control_mode_topic", "/vehicle/status/control_mode");

  min_speed_ = declare_parameter<double>("min_speed", 0.0);
  max_speed_ = declare_parameter<double>("max_speed", 1.6666);
  wheel_base_ = declare_parameter<double>("wheel_base", 0.46);
  max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.7);
  min_angular_speed_ = declare_parameter<double>("min_angular_speed", -0.5);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.5);
  input_gain_ = declare_parameter<double>("input_gain", 1.0);
  deadzone_ = declare_parameter<double>("deadzone", 0.05);
  deadman_button_ = declare_parameter<int>("deadman_button", 2);
  speed_up_button_ = declare_parameter<int>("speed_up_button", 5);
  speed_down_button_ = declare_parameter<int>("speed_down_button", 4);
  speed_axis_ = declare_parameter<int>("speed_axis", 1);
  steering_axis_ = declare_parameter<int>("steering_axis", 3);
  speed_limit_step_ = declare_parameter<double>("speed_limit_step", 0.1);
  speed_limit_factor_ = declare_parameter<double>("initial_speed_limit_factor", 0.5);
  joy_timeout_ = declare_parameter<double>("joy_timeout", 0.5);
  control_timeout_ = declare_parameter<double>("control_timeout", 0.5);

  if (speed_axis_ < 0 || steering_axis_ < 0 || deadman_button_ < 0 ||
    speed_up_button_ < 0 || speed_down_button_ < 0)
  {
    throw std::invalid_argument("Joystick axis and button indices must be non-negative");
  }
  if (min_speed_ < 0.0 || max_speed_ < min_speed_ || wheel_base_ <= 0.0 ||
    max_steering_angle_ <= 0.0 || min_angular_speed_ > 0.0 ||
    max_angular_speed_ < 0.0 || input_gain_ <= 0.0 || deadzone_ < 0.0 || deadzone_ >= 1.0 ||
    speed_limit_step_ <= 0.0 || speed_limit_step_ > 1.0 || joy_timeout_ <= 0.0 ||
    control_timeout_ <= 0.0)
  {
    throw std::invalid_argument("Invalid speed, input scaling, or timeout parameter");
  }

  speed_limit_factor_ = std::clamp(speed_limit_factor_, 0.0, 1.0);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 5);
  control_mode_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(control_mode_topic_, 1);
  autoware_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    control_command_topic_, 1,
    std::bind(&AgilexJoyController::autoware_callback, this, std::placeholders::_1));
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AgilexJoyController::joy_callback, this, std::placeholders::_1));
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&AgilexJoyController::watchdog_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "Vehicle interface ready: manual=%s, autonomous=%s, output=%s",
    joy_topic_.c_str(), control_command_topic_.c_str(), cmd_vel_topic_.c_str());
}

bool AgilexJoyController::button_pressed(const sensor_msgs::msg::Joy & msg, int index) const
{
  return static_cast<std::size_t>(index) < msg.buttons.size() &&
         msg.buttons[static_cast<std::size_t>(index)] != 0;
}

bool AgilexJoyController::has_required_inputs(const sensor_msgs::msg::Joy & msg) const
{
  return static_cast<std::size_t>(speed_axis_) < msg.axes.size() &&
         static_cast<std::size_t>(steering_axis_) < msg.axes.size() &&
         static_cast<std::size_t>(deadman_button_) < msg.buttons.size() &&
         static_cast<std::size_t>(speed_up_button_) < msg.buttons.size() &&
         static_cast<std::size_t>(speed_down_button_) < msg.buttons.size();
}

void AgilexJoyController::publish_stop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

void AgilexJoyController::autoware_callback(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  if (manual_active_) {
    return;
  }

  const double velocity = std::clamp(
    static_cast<double>(msg->longitudinal.velocity), -max_speed_, max_speed_);
  const double steering_angle = std::clamp(
    static_cast<double>(msg->lateral.steering_tire_angle),
    -max_steering_angle_, max_steering_angle_);

  geometry_msgs::msg::Twist command;
  command.linear.x = velocity;
  command.angular.z = std::clamp(
    velocity * std::tan(steering_angle) / wheel_base_,
    min_angular_speed_, max_angular_speed_);
  cmd_vel_pub_->publish(command);

  last_autonomous_command_time_ = now();
  autonomous_command_received_ = true;
}

void AgilexJoyController::joy_callback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  last_joy_time_ = now();
  joy_received_ = true;

  if (!has_required_inputs(*msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Joy message is smaller than the configured axis/button indices; stopping");
    publish_stop();
    manual_active_ = false;
    return;
  }

  const bool speed_up_pressed = button_pressed(*msg, speed_up_button_);
  const bool speed_down_pressed = button_pressed(*msg, speed_down_button_);
  if (speed_up_pressed && !previous_speed_up_pressed_) {
    speed_limit_factor_ = std::min(1.0, speed_limit_factor_ + speed_limit_step_);
    RCLCPP_INFO(get_logger(), "Speed limit: %.0f%%", speed_limit_factor_ * 100.0);
  }
  if (speed_down_pressed && !previous_speed_down_pressed_) {
    speed_limit_factor_ = std::max(0.0, speed_limit_factor_ - speed_limit_step_);
    RCLCPP_INFO(get_logger(), "Speed limit: %.0f%%", speed_limit_factor_ * 100.0);
  }
  previous_speed_up_pressed_ = speed_up_pressed;
  previous_speed_down_pressed_ = speed_down_pressed;

  if (!button_pressed(*msg, deadman_button_)) {
    if (manual_active_) {
      publish_stop();
      autonomous_command_received_ = false;
    }
    manual_active_ = false;
    return;
  }

  auto scale_axis = [this](float raw_value) {
      const double value = std::clamp(static_cast<double>(raw_value) * input_gain_, -1.0, 1.0);
      if (std::abs(value) <= deadzone_) {
        return 0.0;
      }
      const double magnitude = (std::abs(value) - deadzone_) / (1.0 - deadzone_);
      return std::copysign(magnitude, value);
    };

  const double speed_input = scale_axis(msg->axes[static_cast<std::size_t>(speed_axis_)]);
  const double steering_input = scale_axis(msg->axes[static_cast<std::size_t>(steering_axis_)]);

  double linear_speed = 0.0;
  if (speed_input != 0.0) {
    const double magnitude = min_speed_ + std::abs(speed_input) * (max_speed_ - min_speed_);
    linear_speed = std::copysign(magnitude, speed_input);
  }
  const double angular_speed = steering_input >= 0.0 ?
    steering_input * max_angular_speed_ : -steering_input * min_angular_speed_;

  geometry_msgs::msg::Twist command;
  command.linear.x = linear_speed * speed_limit_factor_;
  command.angular.z = angular_speed;
  cmd_vel_pub_->publish(command);
  manual_active_ = true;
}

void AgilexJoyController::watchdog_callback()
{
  if (joy_received_ && manual_active_ && (now() - last_joy_time_).seconds() > joy_timeout_) {
    publish_stop();
    manual_active_ = false;
    RCLCPP_WARN(get_logger(), "Joy input timed out; stopping");
  }

  if (!manual_active_ && autonomous_command_received_ &&
    (now() - last_autonomous_command_time_).seconds() > control_timeout_)
  {
    publish_stop();
    autonomous_command_received_ = false;
    RCLCPP_WARN(get_logger(), "Autoware control command timed out; stopping");
  }

  publish_control_mode();
}

void AgilexJoyController::publish_control_mode()
{
  autoware_vehicle_msgs::msg::ControlModeReport report;
  report.stamp = now();
  report.mode = manual_active_ ?
    autoware_vehicle_msgs::msg::ControlModeReport::MANUAL :
    autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_pub_->publish(report);
}

}  // namespace common_agilex_vehicle

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<common_agilex_vehicle::AgilexJoyController>());
  rclcpp::shutdown();
  return 0;
}
