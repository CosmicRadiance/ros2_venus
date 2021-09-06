// Copyright 2021 Lenove Research Shanghai
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Yulei Qiu ray.yuleiqiu@gmail.com
 * \date    2021-09-06
 *
 */
//----------------------------------------------------------------------

// System
#include <iostream>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

// ROS
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "venus_hardware/venus_position_hardware_interface.hpp"

#define AXIS_JOINT_INDEX_MAX  5
#define DEG_TO_RAD(x) ((x)*M_PI / 180.0)
#define RAD_TO_DEG(x) ((x)*180.0 / M_PI)

#define STEERING_GEAR_RATIO 36

#define RAD_TO_POS(x) ((x / (2 * M_PI)) * STEERING_GEAR_RATIO)
#define POS_TO_RAD(x) ((x * (2 * M_PI)) / STEERING_GEAR_RATIO)

namespace venus_hardware
{
hardware_interface::return_type VenusHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("VenusHardwareInterface"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("VenusHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("VenusHardwareInterface"),
        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("VenusHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(
    rclcpp::get_logger("VenusHardwareInterface"), 
    "VenusHardwareInterface Ready.");
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
VenusHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
VenusHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type VenusHardwareInterface::start()
{
  RCLCPP_INFO(rclcpp::get_logger("VenusHardwareInterface"), "Starting ...please wait...");

  pController_ = ActuatorController::initController();
  Actuator::ErrorsDefine ec;
  uIDArray_ = pController_->lookupActuators(ec);
  if (uIDArray_.size() > 0)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("VenusHardwareInterface"),
      "The connected actuators have been found!");
    if (pController_->enableActuatorInBatch(uIDArray_))
    {
      RCLCPP_INFO(
        rclcpp::get_logger("VenusHardwareInterface"),
        "All actuators have been enabled successfully!");
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("VenusHardwareInterface"),
        "Failed to enable actuators");
    }
    for(size_t k = 0; k < uIDArray_.size(); k++)
    {
      ActuatorController::UnifiedID actuator = uIDArray_.at(k);
      pController_->activateActuatorMode(actuator.actuatorID, Actuator::Mode_Profile_Pos);
      RCLCPP_INFO(
        rclcpp::get_logger("VenusHardwareInterface"),
        "Set the position of actuator %d to zero, be careful.", actuator.actuatorID);
      pController_->setPosition(actuator.actuatorID, 0);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("VenusHardwareInterface"),
      "Connected error code: %x", ec);
      return hardware_interface::return_type::ERROR;
  }

  // set some default values when starting the first time
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
    else
    {
      hw_commands_[i] = hw_states_[i];
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("VenusHardwareInterface"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VenusHardwareInterface::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("VenusHardwareInterface"), "Stopping ...please wait...");

  // Disable all connected actuators
  pController_->disableAllActuators();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("VenusHardwareInterface"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VenusHardwareInterface::read()
{
  RCLCPP_INFO(rclcpp::get_logger("VenusHardwareInterface"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    ActuatorController::UnifiedID actuator = uIDArray_.at(i);
    if (i > 1) hw_states_[i] = pController_->getPosition(uint8_t(actuator.actuatorID+1), true);
    else hw_states_[i] = pController_->getPosition(actuator.actuatorID, true);
    hw_states_[i] = POS_TO_RAD(hw_states_[i]);
    RCLCPP_INFO(
      rclcpp::get_logger("VenusHardwareInterface"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("VenusHardwareInterface"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VenusHardwareInterface::write()
{
  RCLCPP_INFO(rclcpp::get_logger("VenusHardwareInterface"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    ActuatorController::UnifiedID actuator = uIDArray_.at(i);
    if (i > 1) pController_->setPosition(uint8_t(actuator.actuatorID+1), RAD_TO_POS(hw_commands_[i]));
    else if (i == 1)
    {
      pController_->setPosition(actuator.actuatorID, RAD_TO_POS(hw_commands_[i]));
      pController_->setPosition(uint8_t(actuator.actuatorID+1), -1*RAD_TO_POS(hw_commands_[i]));
    }
    else pController_->setPosition(actuator.actuatorID, RAD_TO_POS(hw_commands_[i]));
    RCLCPP_INFO(
      rclcpp::get_logger("VenusHardwareInterface"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("VenusHardwareInterface"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace venus_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  venus_hardware::VenusHardwareInterface, hardware_interface::SystemInterface)
