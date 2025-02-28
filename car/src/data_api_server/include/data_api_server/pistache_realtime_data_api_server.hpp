#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>
#include <pistache/router.h>
#include <pistache/http_headers.h>
#include <pistache/common.h>

#include <std_msgs/msg/string.hpp>
#include <map>
#include <string>
#include <memory>
#include <mutex>
#include <vector>

namespace data_api_server
{

// Forward declaration 
class DataAggregator;

// Custom HTTP Handler: routes requests to aggregator logic
class RealtimeDataAPIHandler : public Pistache::Http::Handler
{
public:
  HTTP_PROTOTYPE(RealtimeDataAPIHandler)

  explicit RealtimeDataAPIHandler(std::shared_ptr<DataAggregator> aggregator);

  void onRequest(const Pistache::Http::Request & request,
                 Pistache::Http::ResponseWriter response) override;

private:
  std::shared_ptr<DataAggregator> aggregator_;
};

// Aggregator that subscribes to ROS topics, collects data, and provides stats
class DataAggregator
{
public:
  DataAggregator(rclcpp::Node::SharedPtr node, const std::string & config_file_path);
  void loadConfiguration();  // E.g., parse an XML or other file specifying topics, rates, etc.

  // Called by subscriber callbacks to store new data
  void updateData(const std::string & topic_name, const std::string & msg_data);

  // Return the last received data for a topic
  std::string getLastValue(const std::string & topic_name);

  // Publish a user-provided message to a topic
  void publishMessage(const std::string & topic_name, const std::string & msg_data);

  // Publish a preset message (identified by some code)
  void publishPreset(const std::string & preset_code);

private:
  rclcpp::Node::SharedPtr node_;
  std::string config_file_path_;
  std::mutex data_mutex_;

  // Maps topic_name -> last received data
  std::map<std::string, std::string> last_data_map_;

  // Example: topic_name -> publisher
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;

  // Subscriptions handle
  struct SubscriptionInfo {
    std::string topic_name;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  };

  std::vector<SubscriptionInfo> subscriptions_;

  // Preset code -> (topic_name, message_content)
  std::map<std::string, std::pair<std::string, std::string>> preset_messages_;

  // Helper to create a subscription for a topic
  void createSubscription(const std::string & topic_name);

  // Helper to create a publisher for a topic
  void createPublisher(const std::string & topic_name);
};

// Node class that wraps the Pistache server and the aggregator
class RealtimeDataAPIServerNode : public rclcpp::Node
{
public:
  RealtimeDataAPIServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RealtimeDataAPIServerNode();

private:
  void initServer();
  void initAggregator();

  // Pistache server instance
  Pistache::Http::Endpoint * server_;

  // The aggregator that manages topic data
  std::shared_ptr<DataAggregator> aggregator_;

  // Configuration file path for aggregator
  std::string config_file_path_;
};

}  // namespace data_api_server
