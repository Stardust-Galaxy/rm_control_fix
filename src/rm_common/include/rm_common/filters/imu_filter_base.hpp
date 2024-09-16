#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <realtime_tools/realtime_publisher.h>


namespace rm_common {
class ImuFilterBase {
public:
    //bool init(XmlRpc::XmlRpcValue& imu_data, const std::string& name);
    void update(rclcpp::Time time, double* accel, double* omega, double* ori, double* accel_cov,double* omega_cov, 
                double* ori_cov, double temp, bool camera_trigger);
    virtual void getOrientation(double& q0, double& q1, double& q2, double& q3) = 0;

protected:
    virtual void initFilter(XmlRpc::XmlRpcValue& imu_data) = 0;
    virtual void resetFilter() = 0;
    virtual void filterUpdate(double ax, double ay, double az, double wx, double wy, double wz, double dt) = 0;
    rclcpp::Time last_update_;
    bool initialized_filter_{false};
    std::string frame_id_;
    rclcpp::Publisher imu_data_pub_;
    rclcpp::Publisher imu_temp_pub_;
    rclcpp::Publisher trigger_time_pub_;
    sensor_msgs::Imu imu_pub_data_;
    sensor_msgs::Tempratrue temp_pub_data_;
    sensor_msgs::TimeReference trigger_time_pub_data_;
};
}// namespace rm_common