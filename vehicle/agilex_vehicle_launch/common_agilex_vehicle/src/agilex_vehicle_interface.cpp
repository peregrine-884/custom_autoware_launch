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

#include "common_agilex_vehicle/agilex_vehicle_interface.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

namespace common_agilex_vehicle
{

AgilexVehicleInterface::AgilexVehicleInterface(const rclcpp::NodeOptions & options)
: Node("agilex_vehicle_interface", options)
{
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  gear_command_topic_ = declare_parameter<std::string>(
    "gear_command_topic", "/control/command/gear_cmd");
  hazard_lights_command_topic_ = declare_parameter<std::string>(
    "hazard_lights_command_topic", "/control/command/hazard_lights_cmd");
  light_command_topic_ = declare_parameter<std::string>("light_command_topic", "/light_control");
  scout_status_topic_ = declare_parameter<std::string>("scout_status_topic", "/scout_status");
  wheel_base_ = declare_parameter<double>("wheel_base", 0.46);
  max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.7);
  stopped_velocity_threshold_ = declare_parameter<double>("stopped_velocity_threshold", 0.01);
  startup_light_command_retry_interval_ =
    declare_parameter<double>("startup_light_command_retry_interval", 0.5);

  if (wheel_base_ <= 0.0 || max_steering_angle_ <= 0.0 || stopped_velocity_threshold_ < 0.0 ||
    startup_light_command_retry_interval_ <= 0.0)
  {
    throw std::invalid_argument("Invalid vehicle status conversion parameter");
  }

  light_command_pub_ =
    create_publisher<scout_msgs::msg::ScoutLightCmd>(light_command_topic_, 1);
  gear_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 1);
  steering_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 1);
  velocity_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1);
  hazard_lights_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", 1);
  gear_command_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    gear_command_topic_, 1,
    std::bind(&AgilexVehicleInterface::gear_command_callback, this, std::placeholders::_1));
  hazard_lights_command_sub_ =
    create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    hazard_lights_command_topic_, 1,
    std::bind(
      &AgilexVehicleInterface::hazard_lights_command_callback, this,
      std::placeholders::_1));
  scout_status_sub_ = create_subscription<scout_msgs::msg::ScoutStatus>(
    scout_status_topic_, 10,
    std::bind(&AgilexVehicleInterface::scout_status_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Status interface ready: %s -> Autoware vehicle status",
    scout_status_topic_.c_str());
}

void AgilexVehicleInterface::gear_command_callback(
  const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  if (msg->command != autoware_vehicle_msgs::msg::GearCommand::NONE &&
    msg->command <= autoware_vehicle_msgs::msg::GearCommand::LOW_2)
  {
    current_gear_ = msg->command;
  }
}

void AgilexVehicleInterface::hazard_lights_command_callback(
  const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
  if (msg->command == HazardLightsCommand::NO_COMMAND) {
    return;
  }

  if (!startup_light_initialization_complete_) {
    pending_hazard_command_ = msg->command;
    return;
  }

  apply_hazard_lights_command(msg->command);
}

void AgilexVehicleInterface::apply_hazard_lights_command(uint8_t command_value)
{
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
  scout_msgs::msg::ScoutLightCmd command;
  command.cmd_ctrl_allowed = true;
  if (command_value == HazardLightsCommand::ENABLE) {
    if (!hazard_requested_ && !light_status_received_) {
      RCLCPP_WARN(
        get_logger(), "No Scout light status received; hazard disable will restore lights to off");
    }
    command.front_mode = scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH;
    command.rear_mode = scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH;
    hazard_requested_ = true;
    light_restore_pending_ = false;
  } else if (command_value == HazardLightsCommand::DISABLE) {
    command.front_mode = saved_front_light_mode_;
    command.front_custom_value = saved_front_custom_value_;
    command.rear_mode = saved_rear_light_mode_;
    command.rear_custom_value = saved_rear_custom_value_;
    hazard_requested_ = false;
    light_restore_pending_ = true;
  } else {
    return;
  }
  light_command_pub_->publish(command);
}

void AgilexVehicleInterface::publish_startup_light_off_command()
{
  scout_msgs::msg::ScoutLightCmd command;
  command.cmd_ctrl_allowed = true;
  command.front_mode = scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF;
  command.front_custom_value = 0;
  command.rear_mode = scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF;
  command.rear_custom_value = 0;
  light_command_pub_->publish(command);
}

void AgilexVehicleInterface::scout_status_callback(
  const scout_msgs::msg::ScoutStatus::ConstSharedPtr msg)
{
  if (!startup_light_initialization_complete_) {
    const bool startup_lights_off =
      msg->light_control_enabled &&
      msg->front_light_state.mode == scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF &&
      msg->rear_light_state.mode == scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF;

    if (startup_lights_off) {
      startup_light_initialization_complete_ = true;
      saved_front_light_mode_ = msg->front_light_state.mode;
      saved_front_custom_value_ = msg->front_light_state.custom_value;
      saved_rear_light_mode_ = msg->rear_light_state.mode;
      saved_rear_custom_value_ = msg->rear_light_state.custom_value;
      light_status_received_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Startup light initialization completed: front and rear lights off");

      const uint8_t pending_command = pending_hazard_command_;
      pending_hazard_command_ = autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND;
      if (pending_command != autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND) {
        apply_hazard_lights_command(pending_command);
      }
    } else {
      const auto current_time = now();
      const bool retry_due = !startup_light_off_command_sent_ ||
        (current_time - last_startup_light_command_time_).seconds() >=
        startup_light_command_retry_interval_;
      if (retry_due) {
        publish_startup_light_off_command();
        startup_light_off_command_sent_ = true;
        last_startup_light_command_time_ = current_time;
      }
    }
  }

  if (startup_light_initialization_complete_ && !hazard_requested_) {
    if (light_restore_pending_) {
      const bool restoration_complete =
        msg->front_light_state.mode == saved_front_light_mode_ &&
        msg->front_light_state.custom_value == saved_front_custom_value_ &&
        msg->rear_light_state.mode == saved_rear_light_mode_ &&
        msg->rear_light_state.custom_value == saved_rear_custom_value_;
      light_restore_pending_ = !restoration_complete;
    } else {
      saved_front_light_mode_ = msg->front_light_state.mode;
      saved_front_custom_value_ = msg->front_light_state.custom_value;
      saved_rear_light_mode_ = msg->rear_light_state.mode;
      saved_rear_custom_value_ = msg->rear_light_state.custom_value;
      light_status_received_ = true;
    }
  }

  autoware_vehicle_msgs::msg::GearReport gear;
  gear.stamp = msg->header.stamp;
  gear.report = current_gear_;
  gear_pub_->publish(gear);

  autoware_vehicle_msgs::msg::VelocityReport velocity;
  velocity.header = msg->header;
  velocity.header.frame_id = base_frame_id_;
  velocity.longitudinal_velocity = static_cast<float>(msg->linear_velocity);
  velocity.lateral_velocity = 0.0F;
  velocity.heading_rate = static_cast<float>(msg->angular_velocity);
  velocity_pub_->publish(velocity);

  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = msg->header.stamp;
  if (std::abs(msg->linear_velocity) > stopped_velocity_threshold_) {
    steering.steering_tire_angle = static_cast<float>(std::clamp(
        std::atan(wheel_base_ * msg->angular_velocity / msg->linear_velocity),
        -max_steering_angle_, max_steering_angle_));
  } else {
    steering.steering_tire_angle = 0.0F;
  }
  steering_pub_->publish(steering);

  autoware_vehicle_msgs::msg::HazardLightsReport hazard_lights;
  hazard_lights.stamp = msg->header.stamp;
  const bool front_flashing =
    msg->front_light_state.mode == scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH;
  const bool rear_flashing =
    msg->rear_light_state.mode == scout_msgs::msg::ScoutLightCmd::LIGHT_BREATH;
  hazard_lights.report = msg->light_control_enabled && front_flashing && rear_flashing ?
    autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE :
    autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
  hazard_lights_pub_->publish(hazard_lights);
}

}  // namespace common_agilex_vehicle

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<common_agilex_vehicle::AgilexVehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
