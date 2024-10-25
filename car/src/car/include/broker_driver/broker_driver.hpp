// broker_driver.hpp
#pragma once

#include <baja_msgs/msg/observation.hpp>
#include <rclcpp/rclcpp.hpp>

namespace broker_driver
{

class BrokerDriver : public rclcpp::Node {
  public:
    BrokerDriver();

  private:
    void observation_callback(const baja_msgs::msg::Observation::SharedPtr msg);

    rclcpp::Subscription<baja_msgs::msg::Observation>::SharedPtr mqtt_sub_;
    rclcpp::Subscription<baja_msgs::msg::Observation>::SharedPtr adc_sub_;
    rclcpp::Publisher<baja_msgs::msg::Observation>::SharedPtr transmit_pub_;
    rclcpp::Publisher<baja_msgs::msg::Observation>::SharedPtr sd_writer_pub_;

    int counter_ = 0;
};

} // namespace broker_driver