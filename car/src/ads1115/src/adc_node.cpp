// adc_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <adc_driver/adc_driver.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<adc_driver::ADCDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}