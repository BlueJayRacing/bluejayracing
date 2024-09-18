// broker_node.cpp
#include <rclcpp/rclcpp.hpp>
#include "broker_driver/broker_driver.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<broker_driver::BrokerDriver>();
    std::cout << "starting ROS2 topic broker" << std::endl;
    std::cout << "entering main loop..." << std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}