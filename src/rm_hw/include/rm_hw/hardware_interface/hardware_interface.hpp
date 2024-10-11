#pragma once

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/hardware_component_info.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>

#include <rm_common/hardware_interface/robot_state_interface.hpp>
#include <rm_common/hardware_interface/rm_imu_sensor_interface.hpp>
#include <rm_common/hardware_interface/actuator_extra_interface.hpp>
#include <rm_common/hardware_interface/tof_radar_interface.hpp>
#include <rm_common/hardware_interface/gpio_interface.hpp>

#include <rm_hw/hardware_interface/can_bus.hpp>
#include <rm_hw/hardware_interface/gpio_manager.hpp>