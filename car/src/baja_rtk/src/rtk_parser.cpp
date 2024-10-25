#include "rtk_corrections.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<baja_rtk::RTKCorrectionsPublisher>();
    while (rclcpp::ok()) {
        node->publish_rtk_corrections();
    }
    rclcpp::shutdown();
    return 0;
}