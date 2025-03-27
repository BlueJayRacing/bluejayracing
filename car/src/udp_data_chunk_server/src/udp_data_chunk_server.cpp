#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <thread>
#include <array>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// Include the ROS 2 custom message header from the baja_msgs package.
#include <baja_msgs/msg/data_chunk.hpp>
#include <baja_msgs/msg/sample.hpp>

// Include Boost.Asio for UDP handling.
#include <boost/asio.hpp>

// Include nanopb headers.
extern "C" {
  #include "pb_decode.h"
  #include "data_chunk.pb.h"  // Generated from your nanopb .proto file.
}

// Include nlohmann::json for JSON formatting.
#include <nlohmann/json.hpp>

using boost::asio::ip::udp;
using json = nlohmann::json;

// Adjust these as necessary.
constexpr uint16_t UDP_PORT = 8888;
constexpr size_t MAX_UDP_BUFFER = 1500;  // bytes

// Callback type to pass decoded nanopb DataChunk from the UDP server to the ROS2 node.
using DataChunkCallback = std::function<void(const DataChunk&)>;

/**
 * @brief UDP server that receives UDP datagrams, decodes them via nanopb,
 * and then calls a user-provided callback with the decoded DataChunk.
 */
class UDPDataChunkServer
{
public:
  UDPDataChunkServer(boost::asio::io_context & io_context, uint16_t port, DataChunkCallback callback)
    : socket_(io_context, udp::endpoint(udp::v4(), port)), callback_(callback)
  {
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
      // Prepare nanopb decoding.
      DataChunk nanopb_chunk = DataChunk_init_zero;
      pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<uint8_t*>(recv_buffer_.data()), bytes_transferred);

      // Attempt to decode the DataChunk.
      if (!pb_decode(&stream, DataChunk_fields, &nanopb_chunk)) {
        RCLCPP_WARN(rclcpp::get_logger("UDPDataChunkServer"), "Failed to decode UDP message: %s", PB_GET_ERROR(&stream));
      } else {
        // Call the callback with the decoded DataChunk.
        callback_(nanopb_chunk);
      }

      // Optionally, send a response back (toggled off below).
      /*
      {
        std::string response = "ACK";
        socket_.async_send_to(
          boost::asio::buffer(response),
          remote_endpoint_,
          [](const boost::system::error_code &, std::size_t) {}
        );
      }
      */
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("UDPDataChunkServer"), "UDP receive error: %s", error.message().c_str());
    }
    // Continue receiving.
    start_receive();
  }

  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  std::array<char, MAX_UDP_BUFFER> recv_buffer_;
  DataChunkCallback callback_;
};

/**
 * @brief ROS2 Node that creates a UDPDataChunkServer to listen for UDP messages,
 * converts nanopb DataChunk to JSON (and logs it), then publishes a ROS2 message.
 */
class UDPDataChunkServerNode : public rclcpp::Node
{
public:
  UDPDataChunkServerNode()
    : Node("udp_data_chunk_server_node")
  {
    // Create a publisher for the custom DataChunk message.
    publisher_ = this->create_publisher<baja_msgs::msg::DataChunk>("data_chunk", 100);

    // Initialize Boost.Asio io_context.
    // (We use a separate thread to run io_context_.run())
    udp_server_ = std::make_shared<UDPDataChunkServer>(
      io_context_,
      UDP_PORT,
      std::bind(&UDPDataChunkServerNode::process_nanopb_datagram, this, std::placeholders::_1)
    );

    // Start the io_context_ in a dedicated thread.
    io_thread_ = std::thread([this]() {
      io_context_.run();
    });

    RCLCPP_INFO(this->get_logger(), "UDPDataChunkServerNode started on UDP port %d", UDP_PORT);
  }

  ~UDPDataChunkServerNode() override
  {
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

private:
  /**
   * @brief Process a decoded nanopb DataChunk.
   * This function converts the nanopb structure into JSON (and logs it) then converts
   * the data into a ROS2 message and publishes it.
   *
   * @param np_chunk The decoded nanopb DataChunk.
   */
   void process_nanopb_datagram(const DataChunk & np_chunk)
   {
     // Convert nanopb DataChunk to JSON.
     json j;
     if (np_chunk.which_chunk_type == DataChunk_fixed_tag) {
       j["chunk_type"] = "fixed";
       j["base_timestamp"] = np_chunk.chunk_type.fixed.base_timestamp;
       j["sample_count"] = np_chunk.chunk_type.fixed.sample_count;
       json samples = json::array();
       for (uint32_t i = 0; i < np_chunk.chunk_type.fixed.sample_count && i < 100; i++) {
         json s;
         s["channel_id"] = np_chunk.chunk_type.fixed.samples[i].channel_id;
         s["value"] = np_chunk.chunk_type.fixed.samples[i].value;
         s["timestamp_delta"] = np_chunk.chunk_type.fixed.samples[i].timestamp_delta;
         samples.push_back(s);
       }
       j["samples"] = samples;
     } else if (np_chunk.which_chunk_type == DataChunk_verbose_tag) {
       j["chunk_type"] = "verbose";
       j["sample_count"] = np_chunk.chunk_type.verbose.sample_count;
       json timestamps = json::array();
       json channel_ids = json::array();
       json values = json::array();
       for (uint32_t i = 0; i < np_chunk.chunk_type.verbose.sample_count && i < 100; i++) {
         timestamps.push_back(np_chunk.chunk_type.verbose.timestamps[i]);
         channel_ids.push_back(np_chunk.chunk_type.verbose.channel_ids[i]);
         values.push_back(np_chunk.chunk_type.verbose.values[i]);
       }
       j["timestamps"] = timestamps;
       j["channel_ids"] = channel_ids;
       j["values"] = values;
     } else {
       j["chunk_type"] = "unknown";
     }
     // Pretty-print JSON.
     std::string json_str = j.dump(4);
    //  RCLCPP_INFO(this->get_logger(), "\nReceived DataChunk JSON:\n%s", json_str.c_str());
   
     // Now convert the nanopb structure to a ROS2 message.
     baja_msgs::msg::DataChunk ros_msg;
     if (np_chunk.which_chunk_type == DataChunk_fixed_tag) {
       ros_msg.chunk_type = 0;  // fixed
       ros_msg.base_timestamp = np_chunk.chunk_type.fixed.base_timestamp;
       ros_msg.fixed_sample_count = np_chunk.chunk_type.fixed.sample_count;
       for (uint32_t i = 0; i < np_chunk.chunk_type.fixed.sample_count && i < 100; i++) {
         baja_msgs::msg::Sample sample;
         sample.channel_id = np_chunk.chunk_type.fixed.samples[i].channel_id;
         sample.value = np_chunk.chunk_type.fixed.samples[i].value;
         sample.timestamp_delta = np_chunk.chunk_type.fixed.samples[i].timestamp_delta;
         ros_msg.fixed_samples.push_back(sample);
       }
     } else if (np_chunk.which_chunk_type == DataChunk_verbose_tag) {
       ros_msg.chunk_type = 1;  // verbose
       ros_msg.verbose_sample_count = np_chunk.chunk_type.verbose.sample_count;
       for (uint32_t i = 0; i < np_chunk.chunk_type.verbose.sample_count && i < 100; i++) {
         ros_msg.verbose_timestamps[i] = np_chunk.chunk_type.verbose.timestamps[i];
         ros_msg.verbose_channel_ids[i] = np_chunk.chunk_type.verbose.channel_ids[i];
         ros_msg.verbose_values[i] = np_chunk.chunk_type.verbose.values[i];
       }
     }
     // Publish the ROS2 message.
     publisher_->publish(ros_msg);
   }
   

  rclcpp::Publisher<baja_msgs::msg::DataChunk>::SharedPtr publisher_;
  boost::asio::io_context io_context_;
  std::shared_ptr<UDPDataChunkServer> udp_server_;
  std::thread io_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Use a multi-threaded executor to support high throughput.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<UDPDataChunkServerNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
