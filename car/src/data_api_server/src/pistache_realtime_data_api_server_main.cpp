#include <rclcpp/rclcpp.hpp>
#include "data_api_server/pistache_realtime_data_api_server.hpp"

int main(int argc, char * argv[])
{
  // Use a MultiThreadedExecutor so ROS2 callbacks and the Pistache server can run concurrently
  rclcpp::init(argc, argv);

  auto node_options = rclcpp::NodeOptions();
  // If desired, set a parameter override for the config file path
  // node_options.append_parameter_override("config_file_path", "path/to/aggregator_config.xml");

  auto node = std::make_shared<data_api_server::RealtimeDataAPIServerNode>(node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
