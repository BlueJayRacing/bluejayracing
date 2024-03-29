#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/data_logger.h"
#include "helpers/ipc_config.h"
#include "baja_live_comm.pb.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main() {
  std::cout << "starting data logger..." << std::endl;
  
  // Open queue
  int rx_queue = StationIPC::open_queue(StationIPC::LOGGER_RX_QUEUE, true);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Main loop
  while (true) {
    std::string msg = StationIPC::get_message(rx_queue); // Blocking
    if (msg == "") {
      continue;
    }
    
    LiveComm live_comm;
    live_comm.ParseFromString(msg);
    std::cout << live_comm.DebugString() << std::endl;
  }
  return EXIT_SUCCESS;
}