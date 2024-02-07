#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <chrono>
#include <vector>

#include "mains/receive_dispatcher.h"
#include "helpers/ipc_config.h"
#include "proto_helpers.h"

// This is the recieve logic for stress testing the XBee. It mocks
// the rx dispatching program

int main()
{
  std::cout << "starting receive dispatcher" << std::endl;
  // Open the queues
  const mqd_t radio_rx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_RX_QUEUE);
  if (radio_rx_queue == -1) {
    std::cout << "Failed to get radio queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }
  
  // Main loop
  int bytes_read_this_window;
  auto start = std::chrono::high_resolution_clock::now();

  while (true) {
    std::string msg = StationIPC::get_message(radio_rx_queue);
    bytes_read_this_window += msg.size();

    auto time_elapsed = std::chrono::high_resolution_clock::now() - start;
    if (time_elapsed.count() >= 30.0) {
      std::cout << bytes_read_this_window << "bytes read in last 30 seconds"  << std::endl;
      bytes_read_this_window = 0;
      auto start = std::chrono::high_resolution_clock::now();
    }
  }
  return 0;
}