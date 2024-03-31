#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"

/* Infinite loop attempting to read data from IPC queue and then write to disk */
int main() {
  std::cout << "starting sd writer..." << std::endl;
  
  // Open queue
  const mqd_t data_queue = BajaIPC::open_queue(CarIPC::BROKER_TO_SD_WRITER_QUEUE, true);
  if (data_queue == -1) {
    std::cout << "Failed to get data queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Main loop
  while (true) {
    std::string msg = BajaIPC::get_message(data_queue); // Blocking
    if (msg == "") {
      continue;
    }
    
    Observation observation;
    observation.ParseFromString(msg);
    std::cout << observation.DebugString() << std::endl;
  }
  return EXIT_SUCCESS;
}