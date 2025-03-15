#include "pistache_hello_world/pistache_hello_world.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>
#include <pistache/common.h>
#include <pistache/cookie.h>
#include <pistache/http_headers.h>
#include <pistache/net.h>
#include <pistache/peer.h>
#include <nlohmann/json.hpp>
#include <sstream>

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
  // Listen on 0.0.0.0:9365
  Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(9365));
  auto opts = Pistache::Http::Endpoint::options()
      .threads(1)
      .maxRequestSize(1024*1024)  // Allow 1MB request size
      .maxResponseSize(1024*1024);
      
  // Create the server endpoint
  server_ = new Pistache::Http::Endpoint(addr);
  
  // Initialize and set up the request handler
  server_->init(opts);
  
  // Create handler with a reference to the logger
  auto handler = std::make_shared<DataHandler>(this->get_logger());
  server_->setHandler(handler);
  
  server_->serveThreaded();
  RCLCPP_INFO(this->get_logger(), "Pistache server listening on port 9365");
}

// DataHandler implementation
DataHandler::DataHandler(rclcpp::Logger logger)
: logger_(logger)
{
}

void DataHandler::onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response)
{
  // Get HTTP method as string
  std::string methodStr;
  switch (request.method()) {
    case Pistache::Http::Method::Get:    methodStr = "GET"; break;
    case Pistache::Http::Method::Post:   methodStr = "POST"; break;
    case Pistache::Http::Method::Put:    methodStr = "PUT"; break;
    case Pistache::Http::Method::Delete: methodStr = "DELETE"; break;
    case Pistache::Http::Method::Options: methodStr = "OPTIONS"; break;
    default:                             methodStr = "UNKNOWN";
  }
  
  RCLCPP_INFO(logger_, "Received %s request for: %s", 
              methodStr.c_str(), 
              request.resource().c_str());
  
  if (request.method() == Pistache::Http::Method::Post) {
    std::string body = request.body();
    RCLCPP_INFO(logger_, "Received data (%zu bytes):", body.size());
    
    if (!body.empty()) {
      try {
        auto json = nlohmann::json::parse(body);
        
        // Pretty print the entire parsed JSON to the log using an indent of 4 spaces
        RCLCPP_INFO(logger_, "Parsed JSON:\n%s", json.dump(4).c_str());
        
        // Optional: if the JSON contains a 'samples' array, print some additional info
        if (json.contains("samples") && json["samples"].is_array()) {
          RCLCPP_INFO(logger_, "  Number of samples: %zu", json["samples"].size());
          int count = 0;
          for (const auto& sample : json["samples"]) {
            if (count++ < 5) {
              RCLCPP_INFO(logger_, "    Sample: %s", sample.dump(4).c_str());
            }
          }
          if (count > 5) {
            RCLCPP_INFO(logger_, "    ... and %d more samples", count - 5);
          }
        }
      } catch (const std::exception& e) {
        // Log raw data if JSON parsing fails
        if (body.size() > 500) {
          RCLCPP_INFO(logger_, "  Raw data (first 500 bytes): %s...", body.substr(0, 500).c_str());
        } else {
          RCLCPP_INFO(logger_, "  Raw data: %s", body.c_str());
        }
      }
    }
    
    nlohmann::json responseJson = {
      {"status", "success"},
      {"message", "Data received successfully"},
      {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
    };
    
    response.headers().add<Pistache::Http::Header::ContentType>(MIME(Application, Json));
    response.send(Pistache::Http::Code::Ok, responseJson.dump());
    RCLCPP_INFO(logger_, "Sent JSON response with OK status");
  } else if (request.method() == Pistache::Http::Method::Get) {
    response.headers().add<Pistache::Http::Header::ContentType>(MIME(Text, Plain));
    response.send(Pistache::Http::Code::Ok, "Server is running. Use POST to send data.\n");
    RCLCPP_INFO(logger_, "Sent plain text response for GET request");
  } else {
    response.headers().add<Pistache::Http::Header::ContentType>(MIME(Text, Plain));
    response.send(Pistache::Http::Code::Method_Not_Allowed, "Method not allowed\n");
    RCLCPP_WARN(logger_, "Method not allowed: %s", methodStr.c_str());
  }
}


}  // namespace pistache_hello_world