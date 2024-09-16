#include "rm_common/filters/imu_filter_base.hpp"

namespace rm_common {

bool ImuFilterBase::init(XmlRpc::XmlRpcValue& imu_data, const std::string& name) {
    frame_id_ = (std::string)imu_data["frame_id"];
    initFilter(imu_data);
    imu_data_pub = this->create_publisher<sensor_msgs::msg::Imu>(name + "/imu_data", 100);

}
}