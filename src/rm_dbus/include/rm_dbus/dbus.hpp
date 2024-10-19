//
// Created by stardust on 2024/10/19.
//

#ifndef BUILD_DBUS_HPP
#define BUILD_DBUS_HPP

#include <cstdint>
#include <rm_msgs/msg/dbus_data.hpp>
#include <rclcpp/rclcpp.hpp>

typedef struct {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s0;
    uint8_t s1;
    int16_t wheel;

    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
    uint16_t key;
}  DBusData_t;

class DBus {
public:
    DBus();
    ~DBus();
    void init(const char* serial);
    void read();
    void getData(rm_msgs::msg::DbusData& d_bus_data) const;
    bool isUpdated() const;
private:
    DBusData_t d_bus_data_{};
    int port_{};
    int16_t buff_[18]{};
    bool is_updated_ = false;
    bool success_ = false;
    void unpack();
};
#endif //BUILD_DBUS_HPP
