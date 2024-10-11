//
// Created by stardust on 2024/10/11.
//

#ifndef BUILD_GPIO_MANAGER_HPP
#define BUILD_GPIO_MANAGER_HPP

#include <xmlrpcpp/XmlRpcValue.h>
#include <fcntl.h>
#include <map>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <rm_common/hardware_interface/gpio_interface.hpp>

namespace rm_hw {
    class GpioManager {
    public:
        explicit GpioManager(rclcpp::Logger logger);
        ~GpioManager();

        void setGpioDirection();
    };
}

#endif //BUILD_GPIO_MANAGER_HPP
