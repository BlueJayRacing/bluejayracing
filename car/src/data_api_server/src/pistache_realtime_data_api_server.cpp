#include "data_api_server/pistache_realtime_data_api_server.hpp"

#include <pistache/router.h>
#include <pistache/http_headers.h>
#include <pistache/endpoint.h>
#include <pistache/http.h>

#include <std_msgs/msg/string.hpp>
#include <tinyxml2.h> // Example if you want to parse XML

namespace data_api_server
{

/***********************
 * RealtimeDataAPIHandler
 ***********************/
RealtimeDataAPIHandler::RealtimeDataAPIHandler(std::shared_ptr<DataAggregator> aggregator)
: aggregator_(aggregator)
{}

void RealtimeDataAPIHandler::onRequest(const Pistache::Http::Request & request,
                                       Pistache::Http::ResponseWriter response)
{
  // Basic routing by URL path or method
  if (request.method() == Pistache::Http::Method::Get)
  {
    // Example: GET /data/<topic>
    // Returns the last value for the specified topic
    auto path = request.resource();
    // Could parse path like /data/some_topic
    if (path.rfind("/data/", 0) == 0) {
      auto topic_name = path.substr(6); // everything after /data/
      auto last_value = aggregator_->getLastValue(topic_name);
      response.send(Pistache::Http::Code::Ok, "Last value for " + topic_name + ": " + last_value);
      return;
    }

    // Fallback
    response.send(Pistache::Http::Code::Not_Found, "GET endpoint not found");
  }
  else if (request.method() == Pistache::Http::Method::Post)
  {
    // Example: POST /publish/<topic> => publish data from body to that topic
    auto path = request.resource();
    if (path.rfind("/publish/", 0) == 0) {
      auto topic_name = path.substr(9); // everything after /publish/
      auto msg_data = request.body();
      aggregator_->publishMessage(topic_name, msg_data);
      response.send(Pistache::Http::Code::Ok, "Published to " + topic_name);
      return;
    }

    // Example: POST /publish/preset/<code> => publish a preset message
    if (path.rfind("/publish/preset/", 0) == 0) {
      auto code = path.substr(16); // everything after /publish/preset/
      aggregator_->publishPreset(code);
      response.send(Pistache::Http::Code::Ok, "Published preset code: " + code);
      return;
    }

    // Fallback
    response.send(Pistache::Http::Code::Not_Found, "POST endpoint not found");
  }
  else
  {
    // Other methods not supported for now
    response.send(Pistache::Http::Code::Method_Not_Allowed, "Method not allowed");
  }
}

/***********************
 * DataAggregator
 ***********************/
DataAggregator::DataAggregator(rclcpp::Node::SharedPtr node, const std::string & config_file_path)
: node_(node),
  config_file_path_(config_file_path)
{
  loadConfiguration();
}

void DataAggregator::loadConfiguration()
{
  // Example approach: parse config_file_path_ (XML) for topics, rates, preset messages, etc.
  // This is a simplified snippet. Adapt to your needs.
  // If you don't have an XML parser at hand, you can skip or use placeholders.

  // For demonstration:
  //   We'll pretend config is something like:
  //   <config>
  //     <topic name="chatter" presetCode="hello_code" presetMsg="Hello from preset" />
  //   </config>

  // Example with tinyxml2 (not shown in CMake, but you can adapt):
  // tinyxml2::XMLDocument doc;
  // if (doc.LoadFile(config_file_path_.c_str()) == tinyxml2::XML_SUCCESS) {
  //   auto root = doc.RootElement();
  //   for (auto topicElem = root->FirstChildElement("topic");
  //        topicElem != nullptr; topicElem = topicElem->NextSiblingElement("topic")) {
  //     std::string topic_name = topicElem->Attribute("name");
  //     std::string code = topicElem->Attribute("presetCode");
  //     std::string msg = topicElem->Attribute("presetMsg");
  //     createSubscription(topic_name);
  //     createPublisher(topic_name);
  //     preset_messages_[code] = {topic_name, msg};
  //   }
  // }

  // For a minimal example, let's just assume "chatter" is a topic:
  std::string topic_name = "chatter";
  createSubscription(topic_name);
  createPublisher(topic_name);

  // Example of storing a single preset for demonstration
  preset_messages_["hello_code"] = { topic_name, "Hello from preset" };
}

void DataAggregator::createSubscription(const std::string & topic_name)
{
  // For a new topic, create a subscriber to std_msgs::msg::String
  auto callback = [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
    updateData(topic_name, msg->data);
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub =
    node_->create_subscription<std_msgs::msg::String>(topic_name, 10, callback);

  SubscriptionInfo info;
  info.topic_name = topic_name;
  info.subscription = sub;
  subscriptions_.push_back(info);
}

void DataAggregator::createPublisher(const std::string & topic_name)
{
  auto pub = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
  publishers_[topic_name] = pub;
}

void DataAggregator::updateData(const std::string & topic_name, const std::string & msg_data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_data_map_[topic_name] = msg_data;
}

std::string DataAggregator::getLastValue(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  auto it = last_data_map_.find(topic_name);
  if (it != last_data_map_.end()) {
    return it->second;
  } else {
    return "[No data received yet]";
  }
}

void DataAggregator::publishMessage(const std::string & topic_name, const std::string & msg_data)
{
  auto it = publishers_.find(topic_name);
  if (it != publishers_.end()) {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = msg_data;
    it->second->publish(*msg);
  }
}

void DataAggregator::publishPreset(const std::string & preset_code)
{
  auto it = preset_messages_.find(preset_code);
  if (it != preset_messages_.end()) {
    const auto & topic_name = it->second.first;
    const auto & content = it->second.second;
    publishMessage(topic_name, content);
  }
}

/***********************
 * RealtimeDataAPIServerNode
 ***********************/
RealtimeDataAPIServerNode::RealtimeDataAPIServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pistache_realtime_data_api_server", options),
  server_(nullptr)
{
  // You can retrieve the config file path from a ROS2 parameter or launch argument
  // For demonstration, hardcode or assume an existing file path
  config_file_path_ = declare_parameter<std::string>("config_file_path", "config/aggregator_config.xml");

  RCLCPP_INFO(get_logger(), "Constructing RealtimeDataAPIServerNode");
  initAggregator();
  initServer();
}

RealtimeDataAPIServerNode::~RealtimeDataAPIServerNode()
{
  if (server_) {
    RCLCPP_INFO(get_logger(), "Shutting down Pistache server...");
    server_->shutdown();
    delete server_;
  }
}

void RealtimeDataAPIServerNode::initAggregator()
{
  aggregator_ = std::make_shared<DataAggregator>(shared_from_this(), config_file_path_);
}

void RealtimeDataAPIServerNode::initServer()
{
  // Listen on 0.0.0.0:8080, for instance
  Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(8080));

  auto opts = Pistache::Http::Endpoint::options()
      .threads(1);

  server_ = new Pistache::Http::Endpoint(addr);
  server_->init(opts);

  auto handler = std::make_shared<RealtimeDataAPIHandler>(aggregator_);
  server_->setHandler(handler);

  // Serve in a non-blocking (threaded) manner
  server_->serveThreaded();

  RCLCPP_INFO(get_logger(), "Pistache Realtime Data API Server listening on port 8080");
}

}  // namespace data_api_server
