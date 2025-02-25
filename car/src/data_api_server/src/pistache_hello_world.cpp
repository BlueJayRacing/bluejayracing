#include "pistache_hello_world/pistache_hello_world.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>
#include <pistache/common.h>
#include <pistache/cookie.h>
#include <pistache/http_headers.h>
#include <pistache/net.h>
#include <pistache/peer.h>

namespace pistache_hello_world
{

ServerNode::ServerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("my_pistache_server_node", options),
  server_(nullptr)
{
  RCLCPP_INFO(this->get_logger(), "Constructing ServerNode");
  initServer();
}

ServerNode::~ServerNode()
{
  if (server_) {
    RCLCPP_INFO(this->get_logger(), "Shutting down Pistache server...");
    server_->shutdown();
    delete server_;
  }
}

void ServerNode::initServer()
{
  // Listen on 0.0.0.0:0365
  Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(9365));

  auto opts = Pistache::Http::Endpoint::options()
      .threads(1);
      // .flags(Pistache::Tcp::Options::InstallSignalHandler);

  // Create the server endpoint
  server_ = new Pistache::Http::Endpoint(addr);

  // Initialize and set up the request handler
  server_->init(opts);
  server_->setHandler(Pistache::Http::make_handler<HelloHandler>());
  server_->serveThreaded();

  RCLCPP_INFO(this->get_logger(), "Pistache server listening on port 9365");
}

}  // namespace my_pistache_lib
