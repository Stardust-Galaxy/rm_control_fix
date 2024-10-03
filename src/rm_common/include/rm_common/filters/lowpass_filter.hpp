#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <rm_msgs/msg/lowpass_data.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
class LowpassFilter : public rclcpp::Node {
public:
    explicit LowpassFilter(const rclcpp::NodeOptions& options);
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
    rclcpp::Duration delta_time_ = rclcpp::Duration::from_seconds(0);
    rclcpp::Publisher<rm_msgs::msg::LowpassData>::SharedPtr publisher_{};
    std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::msg::LowpassData>> realtime_publisher_{};
};