#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "baja_live_comm.pb.h"
#include "helpers/ipc_config.h"
#include "proto_helpers.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int main()
{
  // Open the queues
  const mqd_t radio_rx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_RX_QUEUE);
  if (radio_rx_queue == -1) {
    std::cout << "Failed to get radio queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  const std::vector<mqd_t> subscribed_rx_queues = {
    StationIPC::open_queue(StationIPC::LOGGER_RX_QUEUE),
    StationIPC::open_queue(StationIPC::SIMULATION_RX_QUEUE),
  };
  for (int i = 0; i < subscribed_rx_queues.size(); i++) {
    if (subscribed_rx_queues[i] == -1) {
      std::cout << "Failed to get subscriber queue #" << i << " Errno " << errno << std::endl;
      return EXIT_FAILURE;
    }
  }
  
  // Main loop
  while (true) {
    usleep(100000);
    std::cout << "Dispatcher running" << std::endl;
    try_dispatch_recieved_data(radio_rx_queue, subscribed_rx_queues);
  }
  return 0;
}


// Dispatch the data recieved over radio
void try_dispatch_recieved_data(const mqd_t& radio_rx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  std::string msg = StationIPC::get_message(radio_rx_queue);
  if (msg == "") {
    return;
  }

  // Must dispatch to every subscribed proccess
  std::cout << "Dispatching recieved data" << std::endl;
  for (mqd_t ipc_rx_queue : ipc_rx_queues) {
    
    // As a producer, empty the queue if it's full
    int err = StationIPC::send_message(ipc_rx_queue, msg);
    if (err == StationIPC::QUEUE_FULL) {
      StationIPC::get_message(ipc_rx_queue);
      StationIPC::send_message(ipc_rx_queue, msg);
    }
  }
}