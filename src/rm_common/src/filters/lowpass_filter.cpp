#include "rm_common/filters/lowpass_filter.hpp"


LowpassFilter::LowpassFilter(const rclcpp::NodeOptions& options) : Node("lowpass_filter", options) {
    rclcpp::Node::declare_parameter("cutoff_frequency", 0.0);
    rclcpp::Node::declare_parameter("is_debug", false);
    publisher_ = rclcpp::Node::create_publisher<rm_msgs::LowPassFilter>("lowpass_filter", 10);
    if(is_debug_ == true)
        realtime_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<rm_msgs::LowPassFilter>>(publisher_);
}

LowpassFilter::LowpassFilter(double cutoff_frequency) {
    is_debug_ = false;
    cutoff_frequency_ = cutoff_frequency;
    c_ = 1. / tan(M_PI * cutoff_frequency_ * delta_time_.seconds());
    tan_filt_ = tan(M_PI * cutoff_frequency_ * delta_time_.seconds());
}

void LowpassFilter::input(double in, rclcpp::Time time_stamp) {
    in_[2] = in_[1];
    in_[1] = in_[0];
    in_[0] = in;
    if((last_time_stamp_.seconds() == 0) == false) {
        delta_time_ = time_stamp - last_time_stamp_;
        last_time_stamp = time_stamp;
        if(delta_time_.seconds() == 0) {
            RCLCPP_WARN(get_logger(), "delta time is zero. Skipping this loop. Possible CPU overloaed. Current time :%f"
            , time_stamp.seconds());
        }
        return;
    }
    else {
        last_time_stamp_ = time_stamp;
        return;
    }
    if(cutoff_frequency_ != -1 && cutoff_frequency_ > 0) {
        // Check if tan(_) is really small, could cause c = NaN
        tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_.toSec() / 2.);
        // Avoid tan(0) ==> NaN
        if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
        tan_filt_ = -0.01;
        if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
        tan_filt_ = 0.01;
        c_ = 1 / tan_filt_;
    }

    out_[2] = out_[1];
    out_[1] = out_[0];
    out_[0] = (1 / (1 + c_ * c_ + M_SQRT2 * c_)) * (in_[2] + 2 * in_[1] + in_[0] - (c_ * c_ - M_SQRT2 * c_ + 1) * out_[2] - (2 * c_ * c_ - 2) * out_[1]);
    if(is_debug_ == true) {
        rm_msgs::LowPassFilter msg;
        msg.header.stamp = time_stamp;
        msg.input = in;
        msg.output = out_[0];
        realtime_publisher_->msg_ = msg;
        realtime_publisher_->unlockAndPublish();
    }
}

void LowpassFilter::input(double in) {
    intput(in, rclcpp::Time::now());
}

double Lowpassfilter::output() {
    return out_[0];
}

void LowpassFilter::reset() {
    for(int i = 0;i < 3;i += 1) {
        in_[i] = 0;
        out_[i] = 0;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LowpassFilter)