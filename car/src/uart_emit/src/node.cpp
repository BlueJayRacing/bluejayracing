#include "uart_emit/uart_emit.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car::UartEmit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}