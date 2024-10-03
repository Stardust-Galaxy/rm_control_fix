
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
    std::lock_guard<std::mutex> guard(mutex_);
    for(auto& imu : *data_ptr_.id2imu_data_) {
        imu.second.gyro_updated = false;
        imu.second.accel_updated = false;
    }
    for(const auto& frame_stamp : read_buffer_) {
        can_frame frame = frame_stamp.frame;
        if(data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end()) {
            ActData& act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;

        }
    }
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
            if (-1 < id && id < 4)
            {
                rm_frame0_.data[2 * id] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                rm_frame0_.data[2 * id + 1] = static_cast<uint8_t>(cmd);
                frame0_written= true;
            }
            else if (3 < id && id < 8)
            {
                rm_frame1_.data[2 * (id - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                rm_frame1_.data[2 * (id - 4) + 1] = static_cast<uint8_t>(cmd);
                frame1_written = true;
            }
        } else if(item.second.type.find("cheetah") != std::string::npos) {
            can_frame frame{};
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
            frame.can_id = item.first;
            frame.can_dlc = 8;
            uint16_t q_des = static_cast<int>(act_coeff.pos2act * (item.second.cmd_pos - act_coeff.act2pos_offset));
            uint16_t qd_des = static_cast<int>(act_coeff.vel2act * (item.second.cmd_vel - act_coeff.act2vel_offset));
            uint16_t kp = 0.;
            uint16_t kd = 0.;
            uint16_t tau = static_cast<int>(act_coeff.eff2act * (item.second.exe_effort - act_coeff.act2eff_offset));
            // TODO add position vel and effort hardware interface for MIT Cheetah Motor, now we using it as an effort joint.
            frame.data[0] = q_des >> 8;
            frame.data[1] = q_des & 0xFF;
            frame.data[2] = qd_des >> 4;
            frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
            frame.data[4] = kp & 0xFF;
            frame.data[5] = kd >> 4;
            frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
            frame.data[7] = tau & 0xff;
            socket_can_.write(&frame);
        }
    }
    if(frame0_written)
        socket_can_.write(&rm_frame0_);
    if(frame1_written)
        socket_can_.write(&rm_frame1_);
}


