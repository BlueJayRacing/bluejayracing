// main.cpp
#include <rclcpp/rclcpp.hpp>
#include "mqtt_echo_driver/mqtt_echo_driver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mqtt_echo_driver::MQTTEchoDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}