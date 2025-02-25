#include <rclcpp/rclcpp.hpp>
#include "xbee_driver/xbee_driver.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<xbee_driver::XbeeDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}