#include <iostream>
#include <thread>

#include "station_driver.h"
#include "trx_queues.h"
#include "xbee_connection.h"


TRXProtoQueues* tx_comm_queues;
TRXProtoQueues* rx_comm_queues;

// Digi XBee library global vars. Defined in frame_handling.h
const XBeeConnection* global_xbee_conn_obj = nullptr;
const xbee_dispatch_table_entry_t xbee_frame_handlers[]

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
  delete tx_queues;
  delete rx_queues;
  return 0;
}
