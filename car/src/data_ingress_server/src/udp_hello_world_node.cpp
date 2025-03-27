#include "rclcpp/rclcpp.hpp"
#include "udp_hello_world/udp_hello_world.hpp"
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

class UdpHelloWorldNode : public rclcpp::Node {
public:
  UdpHelloWorldNode()
    : Node("udp_hello_world_node"),
      io_context_(),
      udp_server_(io_context_, udp_port_),
      timer_(io_context_),
      total_bytes_received_(0)
  {
    // Run the Boost.Asio IO context in a separate thread.
    io_thread_ = std::thread([this]() { io_context_.run(); });
    RCLCPP_INFO(this->get_logger(), "UDP Hello World Node started on port %d", udp_port_);

    // Set up a periodic timer to print statistics every 2 seconds.
    start_stats_timer();
  }

  ~UdpHelloWorldNode() {
    io_context_.stop();
    if (io_thread_.joinable())
      io_thread_.join();
  }

private:
  void start_stats_timer() {
    timer_.expires_after(std::chrono::seconds(2));
    timer_.async_wait([this](const boost::system::error_code &error) {
      if (!error) {
        UdpStats stats = udp_server_.getAndResetStats();
        total_bytes_received_ += stats.bytes_received;

        // Calculate data rate in Mbps over the 2 second interval:
        double data_rate_mbps = (stats.bytes_received * 8.0) / (2.0 * 1e6);
        double total_MB = total_bytes_received_ / (1024.0 * 1024.0);

        RCLCPP_INFO(this->get_logger(),
                    "Interval Stats: Received=%lu msgs, Sent=%lu msgs, Interval Volume=%lu bytes, Rate=%.3f Mbps; Aggregated Volume=%.2f MB",
                    stats.messages_received, stats.messages_sent, stats.bytes_received, data_rate_mbps, total_MB);

        start_stats_timer();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Timer error: %s", error.message().c_str());
      }
    });
  }

  boost::asio::io_context io_context_;
  const short udp_port_ = 8888;  // The UDP port used
  UDPHelloWorld udp_server_;
  std::thread io_thread_;
  boost::asio::steady_timer timer_;
  unsigned long total_bytes_received_; // Aggregated total bytes received since startup
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpHelloWorldNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
