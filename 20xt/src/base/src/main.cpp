#include <iostream>
#include <thread>

#include "station_driver.h"
#include "trx_queues.h"

TRXProtoQueues* tx_queues;
TRXProtoQueues* rx_queues;

void xbee_driver_thread(int id) {
  station_main_loop(tx_queues, rx_queues);
}

int main() {

  // Allocate the shared data structures
  tx_queues = new TRXProtoQueues();
  rx_queues = new TRXProtoQueues();

  // Create worker threads
  std::thread xbee_driver(xbee_driver_thread, 1);

  // Wait for worker threads to finish
  xbee_driver.join();

  // Clean up shared datastructures
  delete tx_queues;
  delete rx_queues;
  return 0;
}
