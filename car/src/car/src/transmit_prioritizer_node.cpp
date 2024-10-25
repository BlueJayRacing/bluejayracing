#include "transmit_prioritizer_driver/transmit_prioritizer_driver.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<transmit_prioritizer::TransmitPrioritizerDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}