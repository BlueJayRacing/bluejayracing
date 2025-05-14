//mqtt_client_publish_node
#include <rclcpp/rclcpp.hpp>
#include "wsg_update_time_driver/wsg_update_time_driver.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wsg_update_time_driver::WSGUpdateTimeDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}