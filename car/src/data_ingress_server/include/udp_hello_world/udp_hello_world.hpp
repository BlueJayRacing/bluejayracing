#ifndef UDP_HELLO_WORLD_HPP
#define UDP_HELLO_WORLD_HPP

#include <boost/asio.hpp>
#include <string>

// Structure for UDP statistics.
struct UdpStats {
  unsigned long messages_received;
  unsigned long messages_sent;
  unsigned long bytes_received;   // total bytes received
};

class UDPHelloWorld {
public:
  // Constructor. Opens a UDP socket on the given port.
  UDPHelloWorld(boost::asio::io_context& io_context, short port);
  // Destructor.
  ~UDPHelloWorld();

  // Start the asynchronous receive loop.
  void start_receive();

  // Get and reset statistics.
  UdpStats getAndResetStats();

private:
  // Handlers for receive and send operations.
  void handle_receive(const boost::system::error_code& error,
                      std::size_t bytes_transferred);
  void handle_send(const boost::system::error_code& error,
                   std::size_t bytes_transferred);

  boost::asio::io_context& io_context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];

  // Statistics counters.
  unsigned long messages_received_;
  unsigned long messages_sent_;
  unsigned long total_bytes_received_;
};

#endif  // UDP_HELLO_WORLD_HPP
