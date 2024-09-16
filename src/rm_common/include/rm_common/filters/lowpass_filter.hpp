#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/LowPassFilter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
class LowpassFilter {
public:
    explicit LowpassFilter(rclcpp::NodeOptions& options) : Node("lowpass_filter", options);
    explicit LowpassFilter(double cutoff_frequency);
    void input(double in);
    void input(double in, rclcpp::Time time_stamp);
    double output();
    void reset();
private:
    double in_[3]{};
    double out_[3]{};
    /**
     * @brief -1 means not set
     * 
     */
    double cutoff_frequency_ = -1;
    /**
     * @brief used in filter calc
     * 
     */
    double c_ = 1.;

    double tan_filt_ = 1.;
    /**
     * @brief switch between normal mode and debug mode
     * 
     */
    bool is_debug_ = false;

    rclcpp::Time last_time_stamp_;
    rclcpp::Duration delta_time_;
    rclcpp::Publisher<rm_msgs::LowPassFilter>::SharedPtr publisher_{};
    std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LowPassFilter>> realtime_publisher_{};
};