//
// Created by stardust on 2024/10/12.
//

#ifndef BUILD_ACTUATOR_EXTRA_INTERFACE_HPP
#define BUILD_ACTUATOR_EXTRA_INTERFACE_HPP

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <string>
#include <vector>
#include <initializer_list>

namespace rm_control {
    class ActuatorExtraInterface : public hardware_interface::ActuatorInterface {
    public:
        ActuatorExtraInterface()
                : hardware_interface::ActuatorInterface() {
        }

        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
            if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
                return CallbackReturn::ERROR;
            }

            // 初始化变量
            halted_ = false;
            need_calibration_ = true;
            calibrated_ = false;
            calibration_reading_ = false;
            pos_ = 0.0;
            offset_ = 0.0;

            return CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
            std::vector<hardware_interface::StateInterface> state_interfaces;
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    hardware_interface::InterfaceDescription("halted", hardware_interface::InterfaceInfo{.name ="halted",
                                                                                                          .min = "0",
                                                                                                          .max = "1",
                                                                                                          .initial_value = "0",
                                                                                                          .data_type = "bool"})));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    hardware_interface::InterfaceDescription("need_calibration", hardware_interface::InterfaceInfo{.name = "need_calibration",
                                                                                                                   .min = "0",
                                                                                                                   .max = "1",
                                                                                                                   .initial_value = "0",
                                                                                                                   .data_type = "bool"})));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    hardware_interface::InterfaceDescription("calibrated", hardware_interface::InterfaceInfo{.name = "calibrated",
                                                                                                             .min = "0",
                                                                                                             .max = "1",
                                                                                                             .initial_value = "0",
                                                                                                             .data_type = "bool"})));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    hardware_interface::InterfaceDescription("calibration_reading", hardware_interface::InterfaceInfo{.name = "calibration_reading",
                                                                                                                      .min = "0",
                                                                                                                      .max = "1",
                                                                                                                      .initial_value = "0",
                                                                                                                      .data_type = "bool"})));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    hardware_interface::InterfaceDescription("position", hardware_interface::InterfaceInfo{.name = "position",
                                                                                                           .min = "0",
                                                                                                           .max = "8191",
                                                                                                           .initial_value = "0",
                                                                                                           .data_type = "double"})));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                    hardware_interface::InterfaceDescription("offset", hardware_interface::InterfaceInfo{.name = "offset",
                                                                                                         .min = "0",
                                                                                                         .max = "0",
                                                                                                         .initial_value = "0",
                                                                                                         .data_type = "double"})));
            return state_interfaces;
        }

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
            std::vector<hardware_interface::CommandInterface> command_interfaces;
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    hardware_interface::InterfaceDescription("halted", hardware_interface::InterfaceInfo{.name = "halted",
                                                                                                         .min = "0",
                                                                                                         .max = "1",
                                                                                                         .initial_value = "0",
                                                                                                         .data_type = "bool"})));
            return command_interfaces;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
            // 激活时的逻辑
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
            // 停用时的逻辑
            return CallbackReturn::SUCCESS;
        }
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override
        {
            // 读取硬件状态的逻辑
            // 这里你需要实现实际的硬件读取操作
            return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override
        {
            // 写入硬件命令的逻辑
            // 这里你需要实现实际的硬件写入操作
            return hardware_interface::return_type::OK;
        }

        // 添加一些公共方法来模拟原始 ActuatorExtraHandle 的功能
        std::string getName() const { return info_.name; }
        bool getHalted() const { return halted_; }
        bool getNeedCalibration() const { return need_calibration_; }
        bool getCalibrated() const { return calibrated_; }
        bool getCalibrationReading() const { return calibration_reading_; }
        double getPosition() const { return pos_; }
        double getOffset() const { return offset_; }
        void setOffset(double offset) { offset_ = offset; }
        void setCalibrated(bool calibrated) { calibrated_ = calibrated; }

    private:
        // 状态变量
        bool halted_;
        bool need_calibration_;
        bool calibrated_;
        bool calibration_reading_;
        double pos_;
        double offset_;
    };
}

#endif //BUILD_ACTUATOR_EXTRA_INTERFACE_HPP
