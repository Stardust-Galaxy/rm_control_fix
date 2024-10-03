#pragma once

#include <string>
#include "rm_common/filters/lowpass_filter.hpp"
#include "rm_common/filters/imu_filter_base.hpp"
#include <unordered_map>

namespace rm_hw {

struct ActCoeff {
    double act2pos, act2vel, act2eff, pos2act, vel2act, eff2act, max_out, act2pos_offset, act2vel_offset,
    act2eff_offset, kp2act, kd2act; 
};

struct ActData {
    std::string name;
    std::string type;
    rclcpp::Time stamp;
    uint64_t seq;
    bool halted = false, need_calibration = false, calibrated = false, calibration_reading = false;
    uint16_t q_raw;
    int16_t qd_raw;
    uint8_t temp;
    int64_t q_circle;
    uint16_t q_last;
    double frequency;
    double pos, vel, effort;
    double cmd_pos, cmd_vel, cmd_effort, exe_effort;
    double offset;
    LowpassFilter* lp_filter;
};

struct ImuData {
    rclcpp::Time stamp;
    std::string imu_name;
    double ori[4];
    double angular_vel[3];
    double linear_acc[3];
    double angular_vel_offset[3];
    double ori_cov[9],angular_vel_cov[9],linear_acc_cov[9];
    double temperature, angular_vel_coeff, accel_coeff, temp_coeff, temp_offset;
    bool accel_updated, gyro_updated, camera_trigger;
    bool enabled_trigger;
    rm_common::ImuFilterBase* imu_filter;
};

struct TofData {
    double strength;
    double distance;
};

struct CanDataPtr {
    std::unordered_map<std::string, ActCoeff>* type2act_coeffs_;
    std::unordered_map<int, ActData>* id2act_data_;
    std::unordered_map<int, ImuData>* id2imu_data_;
    std::unordered_map<int, TofData>* id2of_data_;
};
}