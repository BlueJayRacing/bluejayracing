#include <rclcpp/rclcpp.hpp>
#include "pistache_hello_world/pistache_hello_world.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pistache_hello_world::ServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
