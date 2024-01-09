// sub.cpp
// C++ Header files
#include <memory>

// Import the rclcpp client library and std_msgs header
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// creation of function objects with placeholder arguments
using std::placeholders::_1;

// SubscriberNode Composition
//We create a SubscriberNode class that inherits from the rclcpp::Node
class SubscriberNode : public rclcpp::Node
{
  public:
    SubscriberNode()
    : Node("subscriber")
    {
      // Create a subscriber with String message type and queue size is 10
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "string_msg", 10, std::bind(&SubscriberNode::callback, this, _1));
    }

  private:
    // The moment the message available in the queue and 
    // the call back execute the print statement
    void callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  // Create a default single-threaded executor and spin the SubscriberNode node
  rclcpp::spin(std::make_shared<SubscriberNode>());

  // Shutdown the ROS2 communication
  rclcpp::shutdown();
  
  return 0;
}