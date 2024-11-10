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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dynamixel_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace dynamixel_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  double prev_command_pos{0.0};
  double offset{0.0};
  double gear_ratio{0.0};
};

enum class ControlMode
{
  Position = 0,
  Velocity = 1,
  Torque = 2,
  Current = 3,
  ExtendedPosition = 4,
  MultiTurn = 5,
  CurrentBasedPosition = 6,
  PWM = 7,
  None = 8,
};

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode);

  return_type reset_command();

  ControlMode stringToControlMode(const std::string& mode_str);

  void read_internal();
  void sync_read_internal();
  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  // CallbackReturn set_joint_currents();
  CallbackReturn set_joint_params();

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;

  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::None};
  bool use_dummy_{false};
  bool activated_{false};
  bool enable_torque_{false};
  uint8_t protocol_version_{1};
  bool print_once_{true};
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
