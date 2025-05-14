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
#include <iomanip>

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
  try {
    // Listen on 0.0.0.0:9365
    Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(9365));
    
    // Configure endpoint options with address reuse to avoid "Address already in use" errors
    auto opts = Pistache::Http::Endpoint::options()
        .threads(1)
        .maxRequestSize(1024*1024*5)  // Allow 1MB request size
        .maxResponseSize(1024*1024*5)
        .flags(Pistache::Tcp::Options::ReuseAddr);  // Enable address reuse
    
    // Create the server endpoint
    server_ = new Pistache::Http::Endpoint(addr);
    
    // Initialize and set up the request handler with error handling
    try {
      server_->init(opts);
      
      // Create handler with a reference to the logger and node
      auto handler = std::make_shared<DataHandler>(this->get_logger(), this);
      server_->setHandler(handler);
      
      server_->serveThreaded();
      RCLCPP_INFO(this->get_logger(), "Pistache server listening on port 9365");
    } 
    catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize server: %s", e.what());
      // Clean up and rethrow to signal failure
      delete server_;
      server_ = nullptr;
      throw;
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during server initialization: %s", e.what());
    // Don't throw from constructor - this allows the node to remain alive and potentially retry
    RCLCPP_INFO(this->get_logger(), "Server initialization failed - node will continue without HTTP server");
  }
}

// DataHandler implementation
DataHandler::DataHandler(rclcpp::Logger logger, rclcpp::Node* node)
: logger_(logger),
  node_(node),
  stats_(std::make_shared<SharedStats>())
{
  // Set this as the primary instance that will handle stats reporting
  stats_->is_primary_instance.store(true);
  
  // Create a timer that fires every 2 seconds to print statistics
  if (node_ != nullptr) {
    stats_timer_ = node_->create_wall_timer(
      std::chrono::seconds(2),
      [this]() { this->calculateAndPrintStats(); }
    );
  } else {
    RCLCPP_WARN(logger_, "DataHandler created without valid node pointer - statistics reporting disabled");
  }
}

// Alternative constructor with just logger (for compatibility)
DataHandler::DataHandler(rclcpp::Logger logger)
: logger_(logger),
  node_(nullptr),
  stats_(std::make_shared<SharedStats>())
{
  // Not the primary instance
  stats_->is_primary_instance.store(false);
  RCLCPP_WARN(logger_, "DataHandler created with legacy constructor - statistics reporting disabled");
}

// Copy constructor that handles the shared data properly
DataHandler::DataHandler(const DataHandler& other)
: logger_(other.logger_),
  node_(other.node_),
  stats_(other.stats_) // Share the stats
{
  // Only create a timer for this instance if it's the primary and has a valid node
  if (node_ != nullptr && !stats_->is_primary_instance.load()) {
    // This is a secondary instance - it should NOT create a timer
    // to avoid duplicate stat reporting
    RCLCPP_DEBUG(logger_, "DataHandler copy created - secondary instance");
  }
}

// Assignment operator
DataHandler& DataHandler::operator=(const DataHandler& other) {
  if (this != &other) {
    logger_ = other.logger_;
    node_ = other.node_;
    stats_ = other.stats_; // Share the stats
    
    // We don't copy or create timers in assignment
    stats_timer_.reset();
  }
  return *this;
}

// Destructor
DataHandler::~DataHandler() {
  // Clean up timer if it exists
  stats_timer_.reset();
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
  
  RCLCPP_DEBUG(logger_, "Received %s request for: %s", 
              methodStr.c_str(), 
              request.resource().c_str());
  
  if (request.method() == Pistache::Http::Method::Post) {
    std::string body = request.body();
    
    // Track the request size and timestamp
    {
      std::lock_guard<std::mutex> lock(stats_->mutex);
      
      // Record request statistics
      RequestRecord record;
      record.timestamp = std::chrono::system_clock::now();
      record.size = body.size();
      
      stats_->recent_requests.push_back(record);
      stats_->total_requests.fetch_add(1);
      stats_->total_bytes.fetch_add(body.size());
      
      // Clean up old requests
      cleanOldRequests();
    }
    
    nlohmann::json responseJson = {
      {"status", "success"},
      {"message", "Data received successfully"},
      {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
    };
    
    response.headers().add<Pistache::Http::Header::ContentType>(MIME(Application, Json));
    response.send(Pistache::Http::Code::Ok, responseJson.dump());
    RCLCPP_DEBUG(logger_, "Sent JSON response with OK status");
  } else if (request.method() == Pistache::Http::Method::Get) {
    response.headers().add<Pistache::Http::Header::ContentType>(MIME(Text, Plain));
    response.send(Pistache::Http::Code::Ok, "Server is running. Use POST to send data.\n");
    RCLCPP_DEBUG(logger_, "Sent plain text response for GET request");
  } else {
    response.headers().add<Pistache::Http::Header::ContentType>(MIME(Text, Plain));
    response.send(Pistache::Http::Code::Method_Not_Allowed, "Method not allowed\n");
    RCLCPP_WARN(logger_, "Method not allowed: %s", methodStr.c_str());
  }
}

void DataHandler::calculateAndPrintStats() {
  // Check if we have a valid node pointer
  if (node_ == nullptr) {
    return;
  }
  
  // Only the primary instance should print statistics
  if (!stats_->is_primary_instance.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(stats_->mutex);
  
  // Clean out old records first
  cleanOldRequests();
  
  // Get current time
  auto now = std::chrono::system_clock::now();
  
  // Calculate statistics for the last 10 seconds
  size_t count = stats_->recent_requests.size();
  size_t total_size = 0;
  
  for (const auto& record : stats_->recent_requests) {
    total_size += record.size;
  }
  
  // Calculate average message size and rate
  double avg_size = count > 0 ? static_cast<double>(total_size) / count : 0.0;
  double rate = count / 10.0; // Requests per second (we're keeping 10 seconds of data)
  double data_rate = total_size / 10.0; // Bytes per second
  
  // Format time for display
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm now_tm = *std::localtime(&now_time_t);
  std::stringstream time_ss;
  time_ss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
  
  // Print statistics
  RCLCPP_INFO(logger_, 
    "--- STATS [%s] ---", 
    time_ss.str().c_str());
  RCLCPP_INFO(logger_, 
    "Last 10s: %zu requests, %.2f req/s, %.2f KB/s, avg size: %.2f bytes", 
    count, 
    rate, 
    data_rate / 1024.0, 
    avg_size);
  RCLCPP_INFO(logger_,
    "Total since start: %lu requests, %lu MB received",
    stats_->total_requests.load(),
    stats_->total_bytes.load() / (1024 * 1024));
  RCLCPP_INFO(logger_, "------------------------------------");
}

void DataHandler::cleanOldRequests() {
  // No need for lock since this is always called from a method that already has the lock
  
  // Get current time
  auto now = std::chrono::system_clock::now();
  
  // Remove requests older than 10 seconds
  while (!stats_->recent_requests.empty()) {
    auto& oldest = stats_->recent_requests.front();
    auto age = std::chrono::duration_cast<std::chrono::seconds>(now - oldest.timestamp).count();
    
    if (age > 10) {
      stats_->recent_requests.pop_front();
    } else {
      break;
    }
  }
}

}  // namespace pistache_hello_world