
#include "rm_hw/hardware_interface/can_bus.hpp"


rm_hw::CanBus::CanBus(const std::string &bus_name, rm_hw::CanDataPtr data_ptr, int thread_priority, rclcpp::Clock::SharedPtr clock_) : can_bus_logger_(rclcpp::get_logger("can_bus_logger_")), socket_can_logger_(rclcpp::get_logger("socket_can_logger_")), bus_name_(bus_name), data_ptr_(data_ptr),
                                                                                                                                       socket_can_(clock_, socket_can_logger_) {
    while(!socket_can_.open(bus_name,boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && rclcpp::ok()) {
        std::chrono::milliseconds duration(500);
        rclcpp::sleep_for(duration);;
    }
    RCLCPP_INFO(can_bus_logger_, "CAN bus %s opened", bus_name.c_str());
    rm_frame0_.can_id = 0x200;
    rm_frame0_.can_dlc = 8;
    rm_frame1_.can_id = 0x1FF;
    rm_frame1_.can_dlc = 8;
}

void rm_hw::CanBus::read(rclcpp::Time time) {

}

void rm_hw::CanBus::write() {
    bool frame0_written = false, frame1_written = false;
    std::fill(std::begin(rm_frame0_.data), std::end(rm_frame0_.data), 0);
    std::fill(std::begin(rm_frame1_.data), std::end(rm_frame1_.data), 0);
    for(auto& item : *data_ptr_.id2act_data_) {
        if(item.second.type.find("rm") != std::string::npos) {
            if(item.second.halted)
                continue;
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
            int id = item.first - 0x201;
            double cmd =
                    minAbs(act_coeff.eff2act * item.second.exe_effort, act_coeff.max_out);

        }
    }
}
