#include "udp_hello_world/udp_hello_world.hpp"
#include <iostream>

UDPHelloWorld::UDPHelloWorld(boost::asio::io_context& io_context, short port)
  : io_context_(io_context),
    socket_(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
    messages_received_(0),
    messages_sent_(0),
    total_bytes_received_(0)
{
  start_receive();
}

UDPHelloWorld::~UDPHelloWorld() {
  socket_.close();
}

void UDPHelloWorld::start_receive() {
  socket_.async_receive_from(
    boost::asio::buffer(data_, max_length),
    remote_endpoint_,
    [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
      this->handle_receive(error, bytes_transferred);
    }
  );
}

void UDPHelloWorld::handle_receive(const boost::system::error_code& error,
                                   std::size_t bytes_transferred) {
  if (!error || error == boost::asio::error::message_size) {
    messages_received_++;
    total_bytes_received_ += bytes_transferred;
    // (No per-message printing to avoid clutter.)

    // Send a response back.
    std::string response = "Hello from ROS2 UDP Server!";
    socket_.async_send_to(
      boost::asio::buffer(response),
      remote_endpoint_,
      [this](const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
        this->handle_send(error, 0);
      }
    );
    // Continue receiving.
    start_receive();
  } else {
    std::cerr << "Receive error: " << error.message() << std::endl;
  }
}

void UDPHelloWorld::handle_send(const boost::system::error_code& error,
                                std::size_t /*bytes_transferred*/) {
  if (!error) {
    messages_sent_++;
  } else {
    std::cerr << "Send error: " << error.message() << std::endl;
  }
}

UdpStats UDPHelloWorld::getAndResetStats() {
  UdpStats stats { messages_received_, messages_sent_, total_bytes_received_ };
  // Reset counters.
  messages_received_ = 0;
  messages_sent_ = 0;
  total_bytes_received_ = 0;
  return stats;
}
