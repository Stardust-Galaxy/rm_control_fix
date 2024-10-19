//
// Created by stardust on 2024/10/19.
//
#include "rm_hw/hardware_interface/gpio_manager.hpp"

namespace rm_hw {
    GpioManager::GpioManager(rclcpp::Logger logger) : logger_(logger) {}

    GpioManager::~GpioManager() {}

    void GpioManager::setGpioDirection(rm_hw::GpioData gpioData) {
        // Open the GPIOs
        std::string file = "/sys/class/gpio/gpio" + std::to_string(gpioData.pin) + "/direction";
        int fd = open(file.data(), O_WRONLY);
        if(fd == -1) {
            RCLCPP_ERROR(logger_, "Failed to open file %s", file.data());
        } else {
            if(gpioData.type == GpioType::INPUT) {
                if((write(fd, "in", 2) != 2)) {
                    RCLCPP_ERROR(logger_, "Failed to set direction of gpio[%d]", gpioData.pin);
                }
            } else {
                if((write(fd, "out", 3) != 3)) {
                    RCLCPP_ERROR(logger_, "Failed to set direction of gpio[%d]", gpioData.pin);
                }
            }
            close(fd);
        }
    }

    void GpioManager::readGpio() {
        for(auto& gpio : gpio_states_) {
            if(gpio.type == GpioType::INPUT) {
                std::string file = "/sys/class/gpio/gpio" + std::to_string(gpio.pin) + "/value";
                FILE* fp = fopen(file.data(), "r");
                if (fp == nullptr) {
                    RCLCPP_ERROR(logger_, "Failed to open file %s", file.data());
                } else {
                    char state = fgetc(fp);
                    bool value = (state == 0x31);
                    *gpio.value = value;
                    fclose(fp);
                }
            }
        }
    }

    void GpioManager::writeGpio() {
        char buffer[1] = {'1'};
        for(auto& gpio : gpio_commands_) {
            std::string file = "/sys/class/gpio/gpio" + std::to_string(gpio.pin) + "/value";
            int fd = open(file.c_str(), O_WRONLY);
            if(fd == -1) {
                RCLCPP_ERROR(logger_, "Failed to write gpio[%d]/value", gpio.pin);
            } else {
                lseek(fd, 0, SEEK_SET);
                if(gpio.value) {
                    buffer[0] = '1';
                    int ref = write(fd, buffer, 1);
                    if (ref == -1) {
                        RCLCPP_ERROR(logger_, "Failed to write gpio[%d]/value", gpio.pin);
                    }
                } else {
                    buffer[0] = '0';
                    int ref = write(fd, buffer, 1);
                    if (ref == -1) {
                        RCLCPP_ERROR(logger_, "Failed to write gpio[%d]/value", gpio.pin);
                    }
                }
            }
            close(fd);
        }
    }
} // namespace rm_hw