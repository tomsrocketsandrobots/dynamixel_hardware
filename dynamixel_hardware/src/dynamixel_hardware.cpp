// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
// constexpr uint8_t kGoalCurrentIndex = 2;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
// constexpr const char * kGoalCurrentItem = "Goal_Current";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity", "Profile_Acceleration", "Position_P_Gain", "Position_I_Gain",
  "Position_D_Gain",  "Velocity_P_Gain",      "Velocity_I_Gain",
};

ControlMode DynamixelHardware::stringToControlMode(const std::string & mode_str)
{
  if (mode_str == "Position") return ControlMode::Position;
  if (mode_str == "Velocity") return ControlMode::Velocity;
  if (mode_str == "Torque") return ControlMode::Torque;
  if (mode_str == "Current") return ControlMode::Current;
  if (mode_str == "ExtendedPosition") return ControlMode::ExtendedPosition;
  if (mode_str == "MultiTurn") return ControlMode::MultiTurn;
  if (mode_str == "CurrentBasedPosition") return ControlMode::CurrentBasedPosition;
  if (mode_str == "PWM") return ControlMode::PWM;
  if (mode_str == "None") return ControlMode::None;

  RCLCPP_ERROR(
    rclcpp::get_logger(kDynamixelHardware), "You gave an invaid control code: %s",
    mode_str.c_str());
  return ControlMode::Position;
}

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "on_init");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    (info_.hardware_parameters.at("use_dummy") == "true" ||
     info_.hardware_parameters.at("use_dummy") == "True")) {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
  }

  if (info_.hardware_parameters.find("protocol_version") != info_.hardware_parameters.end()) {
    if (info_.hardware_parameters.at("protocol_version") == "1") {
      protocol_version_ = 1;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Protocol Version 1.0");
    } else if (info_.hardware_parameters.at("protocol_version") == "2") {
      protocol_version_ = 2;
      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware), "Protocol Version %f",
        dynamixel_workbench_.getProtocolVersion());
    }
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    dynamixel_workbench_.changeProtocolVersion(joint_ids_[i], protocol_version_);
    // dynamixel_workbench_.set
    joints_[i].offset = std::stod(info_.joints[i].parameters.at("offset"));
    joints_[i].gear_ratio = std::stod(info_.joints[i].parameters.at("gear_ratio"));

    joints_[i].state.position = 0.0;
    joints_[i].state.velocity = 0.0;
    joints_[i].state.effort = 0.0;
    if (use_dummy_) {
      joints_[i].command.position = 0.0;
      joints_[i].command.velocity = 0.0;
      joints_[i].command.effort = 0.0;
    } else {
      joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (use_dummy_) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Dummy mode.");
    // We don't want to config the hardware so return
    return CallbackReturn::SUCCESS;
  }

  if (
    info_.hardware_parameters.find("enable_torque") != info_.hardware_parameters.end() &&
    (info_.hardware_parameters.at("enable_torque") == "false" ||
     info_.hardware_parameters.at("enable_torque") == "False")) {
    enable_torque_ = false;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque is disabled");
  } else {
    enable_torque_ = true;
  }

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Done loading parameters.");
  auto control_mode = stringToControlMode(info_.hardware_parameters.at("control_mode"));
  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint %d model: %d", i, model_number);
    }
  }

  enable_torque(false);
  set_control_mode(control_mode);

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware),
      "Initialized kGoalPositionItem: Address = %d, Data Length = %d for joint ID %d",
      goal_position->address, goal_position->data_length, joint_ids_[0]);
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware),
      "Initialized kMovingSpeedItem: Address = %d, Data Length = %d for joint ID %d",
      goal_velocity->address, goal_velocity->data_length, joint_ids_[0]);
  }

  // const ControlItem * goal_current =
  //   dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalCurrentItem);
  // if (goal_current == nullptr) {
  //   return CallbackReturn::ERROR;
  // }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware),
      "Initialized kPresentPositionItem: Address = %d, Data Length = %d for joint ID %d",
      present_position->address, present_position->data_length, joint_ids_[0]);
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware),
      "Initialized kPresentVelocityItem: Address = %d, Data Length = %d for joint ID %d",
      present_velocity->address, present_velocity->data_length, joint_ids_[0]);
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware),
      "Initialized kPresentLoadItem: Address = %d, Data Length = %d for joint ID %d",
      present_current->address, present_current->data_length, joint_ids_[0]);
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  // control_items_[kGoalCurrentItem] = goal_current;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "kGoalPositionItem handler: %s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "kGoalVelocityItem handler: %s", log);
    return CallbackReturn::ERROR;
  }

  // RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Setting up callbacks. 10");
  // if (!dynamixel_workbench_.addSyncWriteHandler(
  //       control_items_[kGoalCurrentItem]->address, control_items_[kGoalCurrentItem]->data_length,
  //       &log)) {
  //   RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  //   return CallbackReturn::ERROR;
  // }

  uint16_t start_address = std::min(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentVelocityItem]->address);
  uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                         control_items_[kPresentVelocityItem]->data_length + 2;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Doing this thing broke?");
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Finished on_init");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "on_activate");
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].state.position = 0.0;
    joints_[i].state.velocity = 0.0;
    joints_[i].state.effort = 0.0;
  }

  enable_torque(enable_torque_);

  activated_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  activated_ = false;

  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "read");
  if (!activated_) {
    return return_type::OK;
  }

  if (use_dummy_) {
    return return_type::OK;
  }

  read_internal();

  return return_type::OK;
}

void DynamixelHardware::sync_read_internal()
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "read_internal");
  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  // Log the motor IDs and initial states for debugging
  RCLCPP_DEBUG(
    rclcpp::get_logger(kDynamixelHardware), "Motor IDs and initial state vectors initialized:");
  for (size_t i = 0; i < ids.size(); ++i) {
    RCLCPP_DEBUG(
      rclcpp::get_logger(kDynamixelHardware),
      "Joint %zu: ID = %d, Initial Position = %d, Initial Velocity = %d, Initial Current = %d", i,
      ids[i], positions[i], velocities[i], currents[i]);
  }

  const char * log = nullptr;
  if (!dynamixel_workbench_.syncRead(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kDynamixelHardware), "syncRead failed for index %d: %s",
      kPresentPositionVelocityCurrentIndex, log);
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware), "syncRead succeeded for index %d",
      kPresentPositionVelocityCurrentIndex);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentCurrentItem]->address,
        control_items_[kPresentCurrentItem]->data_length, currents.data(), &log)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kDynamixelHardware), "read_internal: kPresentCurrentItem: %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentVelocityItem]->address,
        control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kDynamixelHardware), "read_internal: kPresentVelocityItem: %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
        control_items_[kPresentPositionItem]->address,
        control_items_[kPresentPositionItem]->data_length, positions.data(), &log)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kDynamixelHardware), "read_internal: kPresentPositionItem: %s", log);
  }

  for (uint i = 0; i < ids.size(); i++) {
    if (print_once_) {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(kDynamixelHardware),
        "Raw startup position of the motor["
          << i << "]: " << dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]));
      print_once_ = false;
    }
    joints_[i].state.position =
      (dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]) - joints_[i].offset) /
      joints_[i].gear_ratio;
    joints_[i].state.velocity =
      dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]) / joints_[i].gear_ratio;
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
  }
}
void DynamixelHardware::read_internal()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "read_internal - individual reads version");

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<uint32_t> positions(info_.joints.size(), 0);
  std::vector<uint32_t> velocities(info_.joints.size(), 0);
  std::vector<uint32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  const char * log = nullptr;
  for (size_t i = 0; i < ids.size(); ++i) {
    uint8_t motor_id = ids[i];

    // Log motor ID for each individual read
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "Reading data for motor ID %d", motor_id);

    // Read Present Position
    uint32_t position = 0;
    if (!dynamixel_workbench_.readRegister(
          motor_id, control_items_[kPresentPositionItem]->address,
          control_items_[kPresentPositionItem]->data_length, &position, &log)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kDynamixelHardware), "Failed to read position for motor ID %d: %s",
        motor_id, log);
    } else {
      positions[i] = position;
      RCLCPP_DEBUG(
        rclcpp::get_logger(kDynamixelHardware), "Position for motor ID %d: %u", motor_id, position);
    }

    // Read Present Velocity
    uint32_t velocity = 0;
    if (!dynamixel_workbench_.readRegister(
          motor_id, control_items_[kPresentVelocityItem]->address,
          control_items_[kPresentVelocityItem]->data_length, &velocity, &log)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kDynamixelHardware), "Failed to read velocity for motor ID %d: %s",
        motor_id, log);
    } else {
      velocities[i] = velocity;
      RCLCPP_DEBUG(
        rclcpp::get_logger(kDynamixelHardware), "Velocity for motor ID %d: %u", motor_id, velocity);
    }

    // Read Present Current
    uint32_t current = 0;
    if (!dynamixel_workbench_.readRegister(
          motor_id, control_items_[kPresentCurrentItem]->address,
          control_items_[kPresentCurrentItem]->data_length, &current, &log)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kDynamixelHardware), "Failed to read current for motor ID %d: %s",
        motor_id, log);
    } else {
      currents[i] = current;
      RCLCPP_DEBUG(
        rclcpp::get_logger(kDynamixelHardware), "Current for motor ID %d: %u", motor_id, current);
    }
  }

  // Process the retrieved data as in the original function
  for (uint i = 0; i < ids.size(); i++) {
    if (print_once_) {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(kDynamixelHardware),
        "Raw startup position of the motor["
          << i << "]: "
          << dynamixel_workbench_.convertValue2Radian(ids[i], static_cast<int32_t>(positions[i])));
      print_once_ = false;
    }
    joints_[i].state.position =
      (dynamixel_workbench_.convertValue2Radian(ids[i], static_cast<int32_t>(positions[i])) -
       joints_[i].offset) /
      joints_[i].gear_ratio;
    joints_[i].state.velocity =
      dynamixel_workbench_.convertValue2Velocity(ids[i], static_cast<int32_t>(velocities[i])) /
      joints_[i].gear_ratio;
    joints_[i].state.effort =
      dynamixel_workbench_.convertValue2Current(static_cast<int32_t>(currents[i]));
  }
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "write");
  if (!activated_) {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "write: not activated.");
    return return_type::OK;
  }

  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  if (joints_[0].prev_command_pos != joints_[0].command.position) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(kDynamixelHardware),
      "New Desired Command: pos = "
        << joints_[0].command.position << " effort = " << joints_[0].command.effort
        << " effort command = "
        << dynamixel_workbench_.convertCurrent2Value(
             joint_ids_[0], static_cast<float>(joints_[0].command.effort)));
  }
  joints_[0].prev_command_pos = joints_[0].command.position;

  // Send the command values that coinside with the control mode
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    case ControlMode::CurrentBasedPosition:
      return return_type::ERROR;
      break;
    case ControlMode::None:
      return return_type::OK;
      break;
    default:
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(kDynamixelHardware),
        "Control mode not implemented. Mode = " << static_cast<int>(control_mode_));
      return return_type::ERROR;
      break;
  }
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "enable_torque");
  if (use_dummy_) {
    return return_type::OK;
  }

  const char * log = nullptr;

  if (enabled) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "set_control_mode");
  const char * log = nullptr;

  if (mode == ControlMode::Velocity) {
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    control_mode_ = ControlMode::Velocity;
    set_joint_params();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");

    return return_type::OK;
  }

  if (mode == ControlMode::Position) {
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    control_mode_ = ControlMode::Position;
    set_joint_params();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");

    return return_type::OK;
  }

  if (mode == ControlMode::CurrentBasedPosition) {
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setCurrentBasedPositionControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    control_mode_ = ControlMode::CurrentBasedPosition;
    set_joint_params();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Current Based Position control");

    return return_type::OK;
  }

  if (mode == ControlMode::None) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware), "No control Mode");
    return return_type::OK;
  }
  RCLCPP_FATAL(
    rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
  return return_type::ERROR;
}

return_type DynamixelHardware::reset_command()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "reset_command");
  // Get the current joint position
  if (!use_dummy_) {
    read_internal();
  }

  for (uint i = 0; i < joints_.size(); i++) {
    RCLCPP_INFO(
      rclcpp::get_logger("kDynamixelHardware"), "joints_[%d] starting position: %f", i,
      joints_[i].state.position);
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "set_joint_positions");
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    float command_position =
      (joints_[i].command.position * joints_[i].gear_ratio) + joints_[i].offset;
    commands[i] = dynamixel_workbench_.convertRadian2Value(ids[i], command_position);
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "set_joint_velocities");
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    float command_velocity = joints_[i].command.velocity * joints_[i].gear_ratio;
    commands[i] = dynamixel_workbench_.convertVelocity2Value(ids[i], command_velocity);
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "set_joint_params");
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
