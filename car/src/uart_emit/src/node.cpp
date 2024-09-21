#include <rclcpp/rclcpp.hpp>
#include "uart_emit/uart_emit.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car::UartEmit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}