// main.cpp
#include <rclcpp/rclcpp.hpp>
#include "mqtt_client_driver/mqtt_client_driver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mqtt_client_driver::MQTTClientDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}