// adc_node.cpp
#include <adc_driver/adc_driver.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<adc_driver::ADCDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}