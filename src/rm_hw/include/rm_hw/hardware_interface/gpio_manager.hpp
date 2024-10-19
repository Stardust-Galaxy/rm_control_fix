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

namespace rm_hw {
    enum GpioType
    {
        INPUT,
        OUTPUT
    };

    struct GpioData
    {
        std::string name;
        GpioType type;
        int pin;
        bool* value;
    };

    class GpioManager {
    public:
        explicit GpioManager(rclcpp::Logger logger);
        ~GpioManager();

        void setGpioDirection(rm_hw::GpioData gpioData);
        void readGpio();
        void writeGpio();

        std::vector<rm_hw::GpioData> gpio_states_;
        std::vector<rm_hw::GpioData> gpio_commands_;
    private:
         rclcpp::Logger logger_;
    };
}

#endif //BUILD_GPIO_MANAGER_HPP
