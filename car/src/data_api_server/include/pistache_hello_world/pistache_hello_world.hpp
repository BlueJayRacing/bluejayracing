#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>

namespace pistache_hello_world
{

// Handler class for responding to incoming requests
class HelloHandler : public Pistache::Http::Handler
{
public:
  HTTP_PROTOTYPE(HelloHandler)

  void onRequest(const Pistache::Http::Request&, Pistache::Http::ResponseWriter response) override
  {
    response.send(Pistache::Http::Code::Ok, "Hello, World\n");
  }
};

// Node class that wraps the Pistache server
class ServerNode : public rclcpp::Node
{
public:
  ServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ServerNode();

private:
  void initServer();
  Pistache::Http::Endpoint* server_;
};

}  // namespace my_pistache_lib

