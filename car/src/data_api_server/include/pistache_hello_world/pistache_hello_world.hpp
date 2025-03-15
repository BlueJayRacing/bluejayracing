#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>

namespace pistache_hello_world
{

// Handler class for handling data requests
class DataHandler : public Pistache::Http::Handler 
{
public:
  HTTP_PROTOTYPE(DataHandler)

  explicit DataHandler(rclcpp::Logger logger);

  void onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response) override;

private:
  rclcpp::Logger logger_;
};

// Node class that wraps the Pistache server
class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ServerNode();

private:
  void initServer();
  Pistache::Http::Endpoint* server_;
};

}  // namespace pistache_hello_world