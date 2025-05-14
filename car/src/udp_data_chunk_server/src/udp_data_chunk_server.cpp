#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <thread>
#include <array>
#include <memory>
#include <map>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// Include the ROS 2 custom message headers
#include <baja_msgs/msg/data_chunk.hpp>
#include <baja_msgs/msg/unified_sample.hpp>

// Include Boost.Asio for UDP handling
#include <boost/asio.hpp>

// Include nanopb headers
extern "C" {
  #include "pb_decode.h"
  #include "teensy_data.pb.h"  // Generated from teensy_data.proto file
}

// Include for MAC address utilities
#include <ifaddrs.h>
#include <net/if.h>
#include <netpacket/packet.h>

// Include nlohmann::json for JSON formatting
#include <nlohmann/json.hpp>

// Include ament_index_cpp for package share directory lookup
#include <ament_index_cpp/get_package_share_directory.hpp>

using boost::asio::ip::udp;
using json = nlohmann::json;

// Adjust these as necessary
constexpr uint16_t UDP_PORT = 8888;
constexpr size_t MAX_UDP_BUFFER = 1500;  // bytes
constexpr uint32_t MAX_SEQUENCE_NUM = 65535;

/**
 * @brief Struct to hold device configuration loaded from JSON.
 */
struct DeviceConfig {
    uint8_t device_id;
    std::string device_name;
    std::string mac_address;
    std::unordered_map<uint8_t, uint8_t> channel_mapping;        // internal_id -> ros_id
    std::unordered_map<uint8_t, std::string> semantic_names;       // internal_id -> semantic name
};

/**
 * @brief Pre-defined mappings for internal channel names and descriptions.
 */
const std::map<uint8_t, std::string> internalChannelNames = {
    {0, "AIN0"}, {1, "AIN1"}, {2, "AIN2"}, {3, "AIN3"}, 
    {4, "AIN4"}, {5, "AIN5"}, {6, "AIN6"}, {7, "AIN7"}, 
    {8, "AIN8"}, {9, "AIN9"}, {10, "AIN10"}, {11, "AIN11"}, 
    {12, "AIN12"}, {13, "AIN13"}, {14, "AIN14"}, {15, "AIN15"},
    {16, "DIN0"}, {17, "DIN1"}, {18, "DIN2"}, {19, "DIN3"}, 
    {20, "DIN4"}, {21, "DIN5"},
    {22, "MISC0"}, {23, "MISC1"}, {24, "MISC2"}, {25, "MISC3"}, 
    {26, "MISC4"}, {27, "MISC5"}, {28, "MISC6"}, {29, "MISC7"}
};

const std::map<uint8_t, std::string> channelDescriptions = {
    {0, "Ground reference"},
    {1, "5V reference"},
    {2, "2.5V reference"},
    {3, "2.5V reference (buffered)"},
    {4, "Strain gauge 2"},
    {5, "Strain gauge 1"},
    {6, "Channel 1"},
    {7, "Channel 6"},
    {8, "Channel 2"},
    {9, "Channel 7"},
    {10, "Channel 3"},
    {11, "Channel 8"},
    {12, "Channel 4"},
    {13, "Channel 9"},
    {14, "Channel 5"},
    {15, "Channel 10"},
    {16, "Digital Input 0"},
    {17, "Digital Input 1"},
    {18, "Digital Input 2"},
    {19, "Digital Input 3"},
    {20, "Digital Input 4"},
    {21, "Digital Input 5"},
    {22, "System temperature"},
    {23, "Power supply"},
    {24, "CPU load"},
    {25, "Memory usage"},
    {26, "Misc 4"},
    {27, "Misc 5"},
    {28, "Misc 6"},
    {29, "Misc 7"}
};

// Callback type to pass decoded nanopb DataChunk along with sender info.
using DataChunkCallback = std::function<void(const DataChunk&, const std::string&, const std::string&)>;


// Helper function: lookup MAC address from /proc/net/arp for a given IP.
std::string lookupMacAddress(const std::string & ipAddress) {
  std::ifstream arpFile("/proc/net/arp");
  if (!arpFile.is_open()) {
      return "";
  }
  
  std::string line;
  // Skip header
  std::getline(arpFile, line);
  
  while (std::getline(arpFile, line)) {
      std::istringstream iss(line);
      std::string ip, hwType, flags, mac, mask, device;
      if (!(iss >> ip >> hwType >> flags >> mac >> mask >> device))
          continue;
      if (ip == ipAddress) {
          // Normalize MAC to lowercase.
          std::transform(mac.begin(), mac.end(), mac.begin(), ::tolower);
          return mac;
      }
  }
  return "";
}



/**
 * @brief UDP server that receives datagrams, decodes them using nanopb,
 * and calls a provided callback with the decoded DataChunk.
 */
class UDPDataChunkServer
{
public:
  UDPDataChunkServer(boost::asio::io_context & io_context, uint16_t port, DataChunkCallback callback)
    : socket_(io_context), callback_(callback)
  {
    socket_.open(udp::v4());
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
    socket_.bind(udp::endpoint(udp::v4(), port));
    start_receive();
  }

private:
  void start_receive()
  {
    socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_endpoint_,
      [this](const boost::system::error_code & error, std::size_t bytes_recvd)
      {
        this->handle_receive(error, bytes_recvd);
      }
    );
  }

  void handle_receive(const boost::system::error_code & error, std::size_t bytes_transferred)
  {
    if (!error || error == boost::asio::error::message_size) {
      // Gather sender info.
      std::string sender_ip = remote_endpoint_.address().to_string();
      std::string sender_port = std::to_string(remote_endpoint_.port());
      std::string sender_id = sender_ip + ":" + sender_port;
      
      // Prepare nanopb decoding.
      DataChunk nanopb_chunk = DataChunk_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<uint8_t*>(recv_buffer_.data()), bytes_transferred);

      // Decode the DataChunk.
      if (!pb_decode(&stream, DataChunk_fields, &nanopb_chunk)) {
        RCLCPP_WARN(rclcpp::get_logger("UDPDataChunkServer"), 
                    "Failed to decode UDP message from %s: %s", 
                    sender_id.c_str(), PB_GET_ERROR(&stream));
      } else {
        callback_(nanopb_chunk, sender_ip, sender_port);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("UDPDataChunkServer"), 
                  "UDP receive error: %s", error.message().c_str());
    }
    start_receive();
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  std::array<char, MAX_UDP_BUFFER> recv_buffer_;
  DataChunkCallback callback_;
};

/**
 * @brief ROS2 Node that creates a UDPDataChunkServer to listen for UDP messages,
 * converts the nanopb DataChunk into a unified ROS message (with full device and channel info),
 * and publishes it.
 */
 class UnifiedDataChunkServerNode : public rclcpp::Node
 {
 public:
   UnifiedDataChunkServerNode()
     : Node("unified_data_chunk_server_node"), sequence_number_(0),
       enable_json_logging_(true), enable_channel_stats_(true)
   {
     // Create publisher for unified DataChunk messages.
     publisher_ = this->create_publisher<baja_msgs::msg::DataChunk>("unified_data_chunk", 100);
     
     // Load device configuration from JSON using ament_index_cpp.
     load_configuration();
 
     // Initialize Boost.Asio io_context and start the UDP server.
     udp_server_ = std::make_shared<UDPDataChunkServer>(
       io_context_,
       UDP_PORT,
       std::bind(&UnifiedDataChunkServerNode::process_nanopb_datagram, this, 
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
     );
 
     io_thread_ = std::thread([this]() { io_context_.run(); });
 
     // Optionally report channel statistics periodically.
     if (enable_channel_stats_) {
       stats_timer_ = this->create_wall_timer(
         std::chrono::seconds(10),
         std::bind(&UnifiedDataChunkServerNode::report_channel_stats, this));
     }
 
     RCLCPP_INFO(this->get_logger(), "UnifiedDataChunkServerNode started on UDP port %d", UDP_PORT);
   }
 
   ~UnifiedDataChunkServerNode() override
   {
     io_context_.stop();
     if (io_thread_.joinable()) {
       io_thread_.join();
     }
   }
 
 private:
   /**
    * @brief Load device configuration from a JSON file located in the package share directory.
    */
   void load_configuration() {
     try {
       std::string package_share = ament_index_cpp::get_package_share_directory("car_config");
       std::string config_path = package_share + "/config/car_config.json";
       std::ifstream ifs(config_path);
       if (!ifs.is_open()) {
         RCLCPP_ERROR(this->get_logger(), "Failed to open configuration file: %s", config_path.c_str());
         return;
       }
       json config;
       ifs >> config;
       // Assume configuration JSON has a top-level object "teensy" with "variants" and "default".
       for (const auto& variant : config["teensy"]["variants"].items()) {
         DeviceConfig dev_config;
         dev_config.device_id = variant.value()["device_id"].get<uint8_t>();
         dev_config.device_name = variant.value()["device_name"].get<std::string>();
         // Normalize each provided MAC to lowercase.
         for (const auto& mac : variant.value()["mac_addresses"]) {
           std::string mac_str = mac.get<std::string>();
           std::transform(mac_str.begin(), mac_str.end(), mac_str.begin(), ::tolower);
           dev_config.mac_address = mac_str;  // If multiple MAC addresses exist, you might want to cache all.
           // For this example, we'll use the first MAC address.
           break;
         }
         // Load channel mapping if provided.
         if (variant.value().contains("channel_mapping")) {
           for (auto& [internal_str, ros_id] : variant.value()["channel_mapping"].items()) {
             uint8_t internal_id = static_cast<uint8_t>(std::stoi(internal_str));
             dev_config.channel_mapping[internal_id] = ros_id.get<uint8_t>();
           }
         }
         // Load semantic names if provided.
         if (variant.value().contains("semantic_names")) {
           for (auto& [internal_str, semantic_name] : variant.value()["semantic_names"].items()) {
             uint8_t internal_id = static_cast<uint8_t>(std::stoi(internal_str));
             dev_config.semantic_names[internal_id] = semantic_name.get<std::string>();
           }
         }
         // Store configuration indexed by MAC address.
         device_configs_[dev_config.mac_address] = dev_config;
       }
       RCLCPP_INFO(this->get_logger(), "Loaded %zu device configurations", device_configs_.size());
     } catch (const std::exception& e) {
       RCLCPP_ERROR(this->get_logger(), "Error loading configuration: %s", e.what());
     }
   }
 
   /**
    * @brief Process a decoded nanopb DataChunk received over UDP.
    */
   void process_nanopb_datagram(const DataChunk & np_chunk, 
                                 const std::string& sender_ip,
                                 const std::string& sender_port)
   {
     // Get current time as received timestamp.
     auto now = std::chrono::high_resolution_clock::now();
     uint64_t received_time = std::chrono::duration_cast<std::chrono::microseconds>(
                             now.time_since_epoch()).count();
     
     // Lookup the sender's MAC address using the sender IP with caching.
     std::string device_mac;
     {
       std::lock_guard<std::mutex> lock(ip_mac_cache_mutex_);
       auto it = ip_mac_cache_.find(sender_ip);
       if (it != ip_mac_cache_.end()) {
         device_mac = it->second;
       } else {
         device_mac = lookupMacAddress(sender_ip);
         if (device_mac.empty()) {
           device_mac = "00:00:00:00:00:00";
         }
         ip_mac_cache_[sender_ip] = device_mac;
       }
     }
     
     // Look up device configuration by MAC address.
     DeviceConfig dev_config;
     bool config_found = false;
     // Ensure the MAC from the ARP lookup is in lowercase.
     std::transform(device_mac.begin(), device_mac.end(), device_mac.begin(), ::tolower);
     auto config_it = device_configs_.find(device_mac);
     if (config_it != device_configs_.end()) {
       dev_config = config_it->second;
       config_found = true;
     }
     
     uint8_t device_id = config_found ? dev_config.device_id : 0;
     std::string device_name = config_found ? dev_config.device_name : "Unknown";
     
     if (enable_json_logging_) {
       json j;
       j["sender_ip"] = sender_ip;
       j["sender_port"] = sender_port;
       j["device_mac"] = device_mac;
       j["device_id"] = device_id;
       j["device_name"] = device_name;
       j["sample_count"] = np_chunk.sample_count;
       j["received_time"] = received_time;
       if (np_chunk.sample_count > 0) {
         j["base_timestamp"] = np_chunk.timestamps[0];
       }
       size_t log_count = std::min(static_cast<size_t>(np_chunk.sample_count), static_cast<size_t>(5));
       json samples = json::array();
       for (size_t i = 0; i < log_count; i++) {
         json s;
         s["channel_id"] = np_chunk.internal_channel_ids[i];
         s["timestamp"] = np_chunk.timestamps[i];
         s["value"] = np_chunk.values[i];
         samples.push_back(s);
       }
       j["samples"] = samples;
       if (np_chunk.sample_count > log_count) {
         j["note"] = "Showing only first " + std::to_string(log_count) + " of " + std::to_string(np_chunk.sample_count) + " samples";
       }
       std::string json_str = j.dump(2);
      //  RCLCPP_INFO(this->get_logger(), "\nReceived DataChunk:\n%s", json_str.c_str());
     }
     
     // Build the unified ROS DataChunk message.
     auto ros_chunk = std::make_unique<baja_msgs::msg::DataChunk>();
     ros_chunk->timestamp = received_time;
     ros_chunk->source_id = device_name;
     ros_chunk->sequence_num = sequence_number_++;
     if (sequence_number_ > MAX_SEQUENCE_NUM) {
       sequence_number_ = 0;
     }
     
     // Convert each sample.
     for (uint32_t i = 0; i < np_chunk.sample_count; i++) {
       baja_msgs::msg::UnifiedSample sample;
       sample.device_id = device_id;
       sample.device_name = device_name;
       sample.mac_address = device_mac;
       
       uint8_t internal_id = np_chunk.internal_channel_ids[i];
       sample.internal_channel_id = internal_id;
       
       // Map internal channel ID to ros_channel_id if configured.
       if (config_found && dev_config.channel_mapping.count(internal_id) > 0) {
         sample.ros_channel_id = dev_config.channel_mapping[internal_id];
       } else {
         sample.ros_channel_id = internal_id;
       }
       
       // Set channel names.
       auto it_internal = internalChannelNames.find(internal_id);
       sample.internal_channel_name = (it_internal != internalChannelNames.end()) ? it_internal->second : "Unknown";
       
       if (config_found && dev_config.semantic_names.count(internal_id) > 0) {
         sample.semantic_channel_name = dev_config.semantic_names[internal_id];
       } else {
         auto it_desc = channelDescriptions.find(internal_id);
         sample.semantic_channel_name = (it_desc != channelDescriptions.end()) ? it_desc->second : "Unknown Channel";
       }
       
       sample.recorded_time = np_chunk.timestamps[i];
       sample.received_time = received_time;
       sample.data_value = np_chunk.values[i];
       
       ros_chunk->samples.push_back(sample);
     }
     
     publisher_->publish(std::move(ros_chunk));
   }
  
  /**
   * @brief Report channel statistics periodically.
   */
  void report_channel_stats()
  {
    if (totalSamples_ == 0) return;
    
    RCLCPP_INFO(this->get_logger(), "Channel statistics (from %zu total samples):", totalSamples_);
    size_t adcCount = 0, dinCount = 0, miscCount = 0;
    
    for (const auto& [channelId, count] : channelStats_) {
      if (channelId < 16) adcCount += count;
      else if (channelId < 22) dinCount += count;
      else if (channelId < 30) miscCount += count;
      
      float percentage = (static_cast<float>(count) / totalSamples_) * 100.0f;
      std::string channelName = (internalChannelNames.count(channelId)) ? internalChannelNames.at(channelId) : "Unknown";
      std::string channelDesc = (channelDescriptions.count(channelId)) ? channelDescriptions.at(channelId) : "Unknown";
      
      RCLCPP_INFO(this->get_logger(), "  Channel %3d (%s - %s): %zu samples (%.2f%%)", 
                  channelId, channelName.c_str(), channelDesc.c_str(), count, percentage);
    }
    
    RCLCPP_INFO(this->get_logger(), "Summary by type:");
    RCLCPP_INFO(this->get_logger(), "  ADC channels: %zu samples (%.2f%%)", adcCount, (static_cast<float>(adcCount) / totalSamples_) * 100.0f);
    RCLCPP_INFO(this->get_logger(), "  Digital channels: %zu samples (%.2f%%)", dinCount, (static_cast<float>(dinCount) / totalSamples_) * 100.0f);
    RCLCPP_INFO(this->get_logger(), "  Misc channels: %zu samples (%.2f%%)", miscCount, (static_cast<float>(miscCount) / totalSamples_) * 100.0f);
  }

  // ROS publisher for unified DataChunk messages.
  rclcpp::Publisher<baja_msgs::msg::DataChunk>::SharedPtr publisher_;
  
  // UDP server components.
  boost::asio::io_context io_context_;
  std::shared_ptr<UDPDataChunkServer> udp_server_;
  std::thread io_thread_;
  

  // Cache for mapping sender IPs to MAC addresses for faster lookup.
  std::unordered_map<std::string, std::string> ip_mac_cache_;
  std::mutex ip_mac_cache_mutex_;

  // Device configurations loaded from JSON (keyed by normalized MAC address).
  std::unordered_map<std::string, DeviceConfig> device_configs_;
  
  // Flags (hard-coded here) for logging and statistics.
  bool enable_json_logging_;
  bool enable_channel_stats_;
  
  // Message tracking.
  uint32_t sequence_number_;
  
  // Channel statistics.
  std::map<uint8_t, size_t> channelStats_;
  size_t totalSamples_ = 0;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<UnifiedDataChunkServerNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
