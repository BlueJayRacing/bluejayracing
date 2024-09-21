#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "interfaces/connection.h"
#include "xbee/xbee_connection.h"
#include "xbee/xbee_baja_serial_config.h"
#include <std_msgs/msg/string.hpp>

namespace xbee_driver {

class XbeeDriver : public rclcpp::Node {
public:
    XbeeDriver();
    ~XbeeDriver();

private:
    void timer_callback();
    int try_transmit_data(const std_msgs::msg::String& msg);
    int try_receive_data();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Connection* conn_;
};

} // namespace xbee_driver