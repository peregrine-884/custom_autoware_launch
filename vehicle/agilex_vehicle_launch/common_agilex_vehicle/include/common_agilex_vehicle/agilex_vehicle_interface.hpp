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

#ifndef COMMON_AGILEX_VEHICLE__AGILEX_VEHICLE_INTERFACE_HPP_
#define COMMON_AGILEX_VEHICLE__AGILEX_VEHICLE_INTERFACE_HPP_

#include <string>

#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scout_msgs/msg/scout_light_cmd.hpp>
#include <scout_msgs/msg/scout_status.hpp>

namespace common_agilex_vehicle
{

class AgilexVehicleInterface : public rclcpp::Node
{
public:
  explicit AgilexVehicleInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void gear_command_callback(
    const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void hazard_lights_command_callback(
    const autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void scout_status_callback(const scout_msgs::msg::ScoutStatus::ConstSharedPtr msg);

  std::string base_frame_id_;
  std::string gear_command_topic_;
  std::string hazard_lights_command_topic_;
  std::string light_command_topic_;
  std::string scout_status_topic_;
  double wheel_base_;
  double max_steering_angle_;
  double stopped_velocity_threshold_;

  uint8_t current_gear_{autoware_vehicle_msgs::msg::GearReport::PARK};
  uint8_t saved_front_light_mode_{scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF};
  uint8_t saved_rear_light_mode_{scout_msgs::msg::ScoutLightCmd::LIGHT_CONST_OFF};
  uint8_t saved_front_custom_value_{0};
  uint8_t saved_rear_custom_value_{0};
  bool light_status_received_{false};
  bool hazard_requested_{false};
  bool light_restore_pending_{false};

  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_command_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_command_sub_;
  rclcpp::Subscription<scout_msgs::msg::ScoutStatus>::SharedPtr scout_status_sub_;
  rclcpp::Publisher<scout_msgs::msg::ScoutLightCmd>::SharedPtr light_command_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_pub_;
};

}  // namespace common_agilex_vehicle

#endif  // COMMON_AGILEX_VEHICLE__AGILEX_VEHICLE_INTERFACE_HPP_
