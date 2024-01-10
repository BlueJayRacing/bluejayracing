#include <iostream>
#include <thread>

#include "mains/station_driver.h"
#include "ipc/trx_queues.h"

// Global shared data
TRXProtoQueues* tx_comm_queues;
TRXProtoQueues* rx_comm_queues;

void xbee_driver_thread(int id) {
  station_main_loop(tx_comm_queues, rx_comm_queues);
}

int main() {

  // Allocate the shared data structures
  tx_comm_queues = new TRXProtoQueues();
  rx_comm_queues = new TRXProtoQueues();

  // Create worker threads
  std::thread xbee_driver(xbee_driver_thread, 1);

  // Wait for worker threads to finish
  xbee_driver.join();

  // Clean up shared datastructures
  delete tx_comm_queues;
  delete rx_comm_queues;
  return 0;
}
