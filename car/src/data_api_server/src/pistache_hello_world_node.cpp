#include <rclcpp/rclcpp.hpp>
#include "pistache_hello_world/pistache_hello_world.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Set log level to DEBUG to see all log messages
  auto logger = rclcpp::get_logger("main");
  
  RCLCPP_INFO(logger, "Starting Pistache HTTP server node");
  
  auto node = std::make_shared<pistache_hello_world::ServerNode>();
  
  RCLCPP_INFO(logger, "Spinning node...");
  rclcpp::spin(node);
  
  RCLCPP_INFO(logger, "Shutting down...");
  rclcpp::shutdown();
  return 0;
}
