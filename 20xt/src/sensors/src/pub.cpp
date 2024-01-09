// pub.cpp
// C++ Header files
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Import the rclcpp client library and std_msgs header
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// PublisherNode Composition
// We create a Publisher class that inherits from the rclcpp::Node
class PublisherNode: public rclcpp::Node
{
  public:
    PublisherNode()
    : Node("publisher"), count_(0)
    {
      // Create a Publisher with String message type and queue size is 10
      publisher_ = this->create_publisher<std_msgs::msg::String>("string_msg", 10);

      //Create a timer with 500ms delay
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PublisherNode::callback, this));
    }

  private:
    void callback()
    {
      std_msgs::msg::String message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

      // Publish the message
      publisher_->publish(message);
    }

    // Timer objects allow a node to perform a specific action at a specified rate or at a specific time in the future
    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Variable to count number of message published
    size_t count_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  // Create a default single-threaded executor and spin the PublisherNode node
  rclcpp::spin(std::make_shared<PublisherNode>());

  // Shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}