#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "mains/receive_dispatcher.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"
#include "proto_helpers.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int main()
{
  std::cout << "starting receive dispatcher" << std::endl;
  // Open the queues
  const mqd_t radio_rx_queue = BajaIPC::open_queue(StationIPC::XBEE_DRIVER_RX_QUEUE, true);
  if (radio_rx_queue == -1) {
    std::cout << "Failed to get radio queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  const std::vector<mqd_t> subscribed_rx_queues = {
    BajaIPC::open_queue(StationIPC::LOGGER_RX_QUEUE, false),
    BajaIPC::open_queue(StationIPC::SIMULATION_RX_QUEUE, false),
  };
  for (int i = 0; i < subscribed_rx_queues.size(); i++) {
    if (subscribed_rx_queues[i] == -1) {
      std::cout << "Failed to get subscriber queue #" << i << " Errno " << errno << std::endl;
      return EXIT_FAILURE;
    }
  }
  
  // Main loop
  std::cout << "entering main loop..." << std::endl;
  while (true) {
    try_dispatch_recieved_data(radio_rx_queue, subscribed_rx_queues); // Blocking
  }
  return 0;
}


/* Check the radio rx queue for incoming data, copy and dispatch to subscribers */
void try_dispatch_recieved_data(const mqd_t& radio_rx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  std::string msg = BajaIPC::get_message(radio_rx_queue); // Blocking
  if (msg == "") {
    return;
  }

  // Must dispatch to every subscribed proccess
  std::cout << "Dispatching recieved data" << std::endl;
  for (mqd_t ipc_rx_queue : ipc_rx_queues) {
    
    // As a producer, empty the queue if it's full
    int err = BajaIPC::send_message(ipc_rx_queue, msg);
    if (err == BajaIPC::QUEUE_FULL) {
      BajaIPC::get_message(ipc_rx_queue);
      BajaIPC::send_message(ipc_rx_queue, msg);
    }
  }
}