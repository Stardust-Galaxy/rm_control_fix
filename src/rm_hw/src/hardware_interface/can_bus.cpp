
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
            if((frame_stamp.stamp - act_data.stamp).seconds() < 0.0005)
                continue;
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
            if (act_data.type.find("rm") != std::string::npos) {
                act_data.q_raw = (frame.data[0] << 8u) | frame.data[1];
                act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
                int16_t cur = (frame.data[4] << 8u) | frame.data[5];
                act_data.temp = frame.data[6];

                // Multiple circle
                if (act_data.seq != 0)  // not the first receive
                {
                    if (act_data.q_raw - act_data.q_last > 4096)
                        act_data.q_circle--;
                    else if (act_data.q_raw - act_data.q_last < -4096)
                        act_data.q_circle++;
                }
                try
                {  // Duration will be out of dual 32-bit range while motor failure
                    act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).seconds();
                }
                catch (std::runtime_error& ex) {}
                act_data.stamp = frame_stamp.stamp;
                act_data.seq++;
                act_data.q_last = act_data.q_raw;
                // Converter raw CAN data to position velocity and effort.
                act_data.pos =
                        act_coeff.act2pos * static_cast<double>(act_data.q_raw + 8191 * act_data.q_circle) + act_data.offset;
                act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                act_data.effort = act_coeff.act2eff * static_cast<double>(cur);
                // Low pass filter
                act_data.lp_filter->input(act_data.vel, frame_stamp.stamp);
                act_data.vel = act_data.lp_filter->output();
                continue;
            }
            // Check MIT Cheetah motor
        else if (frame.can_id == static_cast<unsigned int>(0x000)) {
            if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
            {
                ActData& act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
                const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
                if (act_data.type.find("cheetah") != std::string::npos)
                {  // MIT Cheetah Motor
                    act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
                    uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
                    uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];
                    // Multiple cycle
                    // NOTE: The raw data range is -4pi~4pi
                    if (act_data.seq != 0)  // not the first receive
                    {
                        double pos_new = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                                         static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
                        if (pos_new - act_data.pos > 4 * M_PI)
                            act_data.q_circle--;
                        else if (pos_new - act_data.pos < -4 * M_PI)
                            act_data.q_circle++;
                    }
                    try
                    {  // Duration will be out of dual 32-bit range while motor failure
                        act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).seconds();
                    }
                    catch (std::runtime_error& ex)
                    {
                    }
                    act_data.stamp = frame_stamp.stamp;
                    act_data.seq++;
                    act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                                   static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
                    // Converter raw CAN data to position velocity and effort.
                    act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.act2vel_offset;
                    act_data.effort = act_coeff.act2eff * static_cast<double>(cur) + act_coeff.act2eff_offset;
                    // Low pass filter
                    act_data.lp_filter->input(act_data.vel);
                    act_data.vel = act_data.lp_filter->output();
                    continue;
                }
            }
        }
        else if (data_ptr_.id2imu_data_->find(frame.can_id) != data_ptr_.id2imu_data_->end())  {// Check if IMU gyro
            ImuData& imu_data = data_ptr_.id2imu_data_->find(frame.can_id)->second;
            imu_data.gyro_updated = true;
            imu_data.angular_vel[0] = (((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.angular_vel_coeff) +
                                      imu_data.angular_vel_offset[0];
            imu_data.angular_vel[1] = (((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.angular_vel_coeff) +
                                      imu_data.angular_vel_offset[1];
            imu_data.angular_vel[2] = (((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.angular_vel_coeff) +
                                      imu_data.angular_vel_offset[2];
            imu_data.time_stamp = frame_stamp.stamp;
            int16_t temp = (int16_t)((frame.data[6] << 3) | (frame.data[7] >> 5));
            if (temp > 1023)
                temp -= 2048;
            imu_data.temperature = temp * imu_data.temp_coeff + imu_data.temp_offset;
            continue;
        }
        else if (data_ptr_.id2imu_data_->find(frame.can_id - 1) != data_ptr_.id2imu_data_->end())  {// Check if IMU accel
            ImuData& imu_data = data_ptr_.id2imu_data_->find(frame.can_id - 1)->second;
            imu_data.accel_updated = true;
            imu_data.linear_acc[0] = ((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.accel_coeff;
            imu_data.linear_acc[1] = ((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.accel_coeff;
            imu_data.linear_acc[2] = ((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.accel_coeff;
            imu_data.time_stamp = frame_stamp.stamp;
            imu_data.camera_trigger = frame.data[6] & 1;
            imu_data.enabled_trigger = frame.data[6] & 2;
            imu_data.imu_filter->update(frame_stamp.stamp, imu_data.linear_acc, imu_data.angular_vel, imu_data.ori,
                                        imu_data.linear_acc_cov, imu_data.angular_vel_cov, imu_data.ori_cov,
                                        imu_data.temperature, imu_data.camera_trigger && imu_data.enabled_trigger);
            continue;
        }
        else if (data_ptr_.id2tof_data_->find(frame.can_id) != data_ptr_.id2tof_data_->end()) {
            TofData& tof_data = data_ptr_.id2tof_data_->find(frame.can_id)->second;
            tof_data.distance = ((int16_t)((frame.data[1]) << 8) | frame.data[0]);
            tof_data.strength = ((int16_t)((frame.data[3]) << 8) | frame.data[2]);
            continue;
        }
        if (frame.can_id != 0x0)
            RCLCPP_ERROR_STREAM_ONCE(can_bus_logger_, "Can not find defined device, id: 0x" << std::hex << frame.can_id
                                                                        << " on bus: " << bus_name_);
        }
    }
    read_buffer_.clear();
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

void rm_hw::CanBus::write(can_frame* frame) {
    socket_can_.write(frame);
}

void rm_hw::CanBus::frameCallback(const can_frame &frame) {
    std::lock_guard<std::mutex> guard(mutex_);
    CanFrameStamp frame_stamp;
    frame_stamp.frame = frame;
    frame_stamp.stamp = rclcpp::Clock().now();
    read_buffer_.push_back(frame_stamp);
}


