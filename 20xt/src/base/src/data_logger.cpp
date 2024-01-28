#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/data_logger.h"
#include "helpers/ipc_config.h"
#include "baja_live_comm.pb.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main() {
  // Open queue
  int rx_queue = StationIPC::open_queue(StationIPC::LOGGER_RX_QUEUE);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Main loop
  while (true) {
    usleep(100000);
    std::cout << "Logger is running" << std::endl;
    std::string msg = StationIPC::get_message(rx_queue);
    if (msg == "") {
      continue;
    }

    Observation data;
    data.ParseFromString(msg); 
    std::cout << "Recieved data: " << data.DebugString() << std::endl;
  }
  return EXIT_SUCCESS;
}