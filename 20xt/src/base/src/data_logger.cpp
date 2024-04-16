#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/data_logger.h"
#include "ipc_config.h"
#include "baja_live_comm.pb.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main() {
  std::cout << "starting data logger..." << std::endl;
  
  // Open queue
  const mqd_t rx_queue = BajaIPC::open_queue(StationIPC::RECEIVE_DISPATCHER_TO_DATA_LOGGER, true);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Main loop
  while (true) {
    std::string msg = BajaIPC::get_message(rx_queue); // Blocking
    if (msg == "") {
      continue;
    }
    
    Observation observation;
    observation.ParseFromString(msg);
    
    std::string json_string;
    google::protobuf::util::MessageToJsonString(observation, &json_string);
    std::cout << json_string << std::endl;
  }
  return EXIT_SUCCESS;
}