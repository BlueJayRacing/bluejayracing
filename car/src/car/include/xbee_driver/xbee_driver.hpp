#pragma once

#include "interfaces/connection.h"
#include "xbee/xbee_baja_serial_config.h"
#include "xbee/xbee_connection.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace xbee_driver
{

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