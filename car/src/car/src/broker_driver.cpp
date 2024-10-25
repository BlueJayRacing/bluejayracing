// broker_driver.cpp
#include "broker_driver/broker_driver.hpp"

namespace broker_driver
{

BrokerDriver::BrokerDriver() : Node("broker_driver")
{
    mqtt_sub_ = create_subscription<baja_msgs::msg::Observation>(
        "mqtt_data", rclcpp::QoS(1000).best_effort().durability_volatile(),
        std::bind(&BrokerDriver::observation_callback, this, std::placeholders::_1));

    adc_sub_ = create_subscription<baja_msgs::msg::Observation>(
        "adc_data", 10, std::bind(&BrokerDriver::observation_callback, this, std::placeholders::_1));

    transmit_pub_  = create_publisher<baja_msgs::msg::Observation>("transmit_data", 10);
    sd_writer_pub_ = create_publisher<baja_msgs::msg::Observation>("sd_writer_data", 10);
}

void BrokerDriver::observation_callback(const baja_msgs::msg::Observation::SharedPtr msg)
{
    // The radio can't handle all of it anyways, TODO: Should we subsample to increase performance?
    if (counter_++ == 200) {
        transmit_pub_->publish(*msg);
        counter_ = 0;
    }
    sd_writer_pub_->publish(*msg);
}

} // namespace broker_driver