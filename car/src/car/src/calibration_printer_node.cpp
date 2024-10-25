#include "calibration_printer/calibration_printer.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car::AxleTorqueAggregator>("calibration_printer");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}