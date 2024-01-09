// Import the rclcpp client library
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);
  
  // Create a ROS2 node named hello_world
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("hello_world");

  // We create a Rate object of 1Hz
  rclcpp::Rate loop_rate(1);

  // Loop until node is shutdown
  while (rclcpp::ok()) {

    // Print a message to the terminal
    RCLCPP_INFO(node->get_logger(), "Hello World......");
    
    // sleep to maintan the Rate we set above
    loop_rate.sleep();    
  }

  // Create a default single-threaded executor and spin the specified node
  rclcpp::spin(node);

  // Shutdown the ROS2 communication
  rclcpp::shutdown();

  return 0;
}