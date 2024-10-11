#pragma once

#include "rm_hw/hardware_interface/socketcan.hpp"
#include "rm_hw/hardware_interface/types.hpp"

#include <chrono>
#include <mutex>
#include <thread>
#include <boost/bind.hpp>
#include <rm_common/utilities/math_utilities.hpp>

namespace rm_hw {
    struct CanFrameStamp {
        can_frame frame;
        rclcpp::Time stamp;
    };

    class CanBus {
    public:
        /**
         * \brief
         * Initialize device at can_device, retry if fail
         */
         CanBus(const std::string& bus_name, CanDataPtr data_ptr, int thread_priority, rclcpp::Clock::SharedPtr  clock_);
         void read(rclcpp::Time time);
         void write();
         void write(can_frame* frame);

         const std::string bus_name_;

    private:
        /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector: read_buffer_.
         *
         * @param frame The frame which socketcan receive.
         */
        void frameCallback(const can_frame& frame);

        can::SocketCAN socket_can_;
        CanDataPtr data_ptr_;
        std::vector<CanFrameStamp> read_buffer_;

        can_frame rm_frame0_{};  // for id 0x201~0x204
        can_frame rm_frame1_{};  // for id 0x205~0x208

        rclcpp::Logger can_bus_logger_;
        rclcpp::Logger socket_can_logger_;
        mutable std::mutex mutex_;

    };
}