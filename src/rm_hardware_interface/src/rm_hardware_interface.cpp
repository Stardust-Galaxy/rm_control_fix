// Copyright (c) 2024, stardust
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>

#include "rm_hardware_interface/rm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_hardware_interface
{
hardware_interface::CallbackReturn RMHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
      if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      {
          return CallbackReturn::ERROR;
      }

      // TODO(anyone): read parameters and initialize the hardware
      actuator_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
      actuator_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
      gpio_states_.resize(info_.gpios.size(),1);
      return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RMHardwareInterface::export_state_interfaces()
{
      std::vector<hardware_interface::StateInterface> state_interfaces;
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &actuator_states_[i]));
      }
      for(size_t i = 0; i < info_.gpios.size(); ++i)
      {
          state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.gpios[i].name, "bool", &gpio_states_[i]));
      }
      return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RMHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &actuator_commands_[i]));
    }
    for(size_t i = 0; i < info_.gpios.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios[i].name, "bool", &gpio_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn RMHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RMHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // TODO(anyone): read robot states

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RMHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'

  return hardware_interface::return_type::OK;
}

}  // namespace rm_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rm_hardware_interface::RMHardwareInterface, hardware_interface::SystemInterface)
