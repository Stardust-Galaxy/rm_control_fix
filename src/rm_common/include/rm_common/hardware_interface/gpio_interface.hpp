//
// Created by stardust on 2024/10/11.
//

#ifndef BUILD_GPIO_INTERFACE_HPP
#define BUILD_GPIO_INTERFACE_HPP

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
namespace rm_control {
    enum class GpioType {
        INPUT,
        OUTPUT
    };
    struct GpioData {
        std::string name;
        GpioType type;
        int pin;
        bool* value;
    };
    class GpioStateHandle {
    public:
        GpioStateHandle() = default;
        hardware_interface::return_type GpioStateHandleInit(const std::string& name, GpioType type, bool* value)  {
            name_ = name;
0 4           type_ = type;
            value_ = value;
            if(value_ == nullptr) {
                return hardware_interface::return_type::ERROR;
            }
        }
    private:
        std::string name_;
        GpioType type_;
        bool* value_;
    };

    class GpioCommandHandle {
    public:
        GpioCommandHandle() = default;
        hardware_interface::return_type GpioCommandHandleInit(const std::string& name, GpioType type, bool* cmd) {
            name_ = name;
            type_ = type;
            cmd_ = cmd;
        }
    };
}
#endif //BUILD_GPIO_INTERFACE_HPP
