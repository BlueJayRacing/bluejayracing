#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>
#include <chrono>
#include <vector>
#include <mutex>
#include <deque>
#include <memory>
#include <atomic>

namespace pistache_hello_world
{

// Request statistics tracking
struct RequestRecord {
  std::chrono::system_clock::time_point timestamp;
  size_t size;
};

// Shared statistics data structure to be used across all handler instances
struct SharedStats {
  std::deque<RequestRecord> recent_requests;
  std::mutex mutex;
  std::atomic<uint64_t> total_requests{0};
  std::atomic<uint64_t> total_bytes{0};
  
  // Primary instance flag to avoid duplicate timers
  std::atomic<bool> is_primary_instance{false};
};

// Handler class for handling data requests
class DataHandler : public Pistache::Http::Handler 
{
public:
  HTTP_PROTOTYPE(DataHandler)

  // Constructor with both logger and node parameters
  explicit DataHandler(rclcpp::Logger logger, rclcpp::Node* node);
  
  // Alternative constructor with just logger for compatibility
  explicit DataHandler(rclcpp::Logger logger);
  
  // Copy constructor that shares data
  DataHandler(const DataHandler& other);
  
  // Assignment operator
  DataHandler& operator=(const DataHandler& other);
  
  // Destructor
  ~DataHandler();

  void onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response) override;

private:
  rclcpp::Logger logger_;
  rclcpp::Node* node_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  // Statistics tracking - shared between all instances
  std::shared_ptr<SharedStats> stats_;
  
  // Calculate and print statistics based on the last 10 seconds
  void calculateAndPrintStats();
  
  // Clean up requests older than 10 seconds
  void cleanOldRequests();
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
