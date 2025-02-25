#include <rclcpp/rclcpp.hpp>
#include "mock_data_api_server/mock_data_api_server.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create a node with default options
  auto node = std::make_shared<mock_data_api::MockDataAPIServerNode>();
  
  // Use a MultiThreadedExecutor for better performance with the Pistache server
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // Spin (blocks until ROS2 is shutdown)
  executor.spin();
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}