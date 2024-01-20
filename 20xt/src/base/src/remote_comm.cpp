#include <iostream>
#include <thread>
#include <mqueue.h>

#include "ipc_config.h"
#include "mains/station_driver.h"
#include "mains/trx_dispatcher.h"
#include "ipc/trx_queues.h"

// GLobal Constants
static const int MAX_QUEUE_SIZE = 300;

// Global shared data
TRXProtoQueues* shared_tx_queue;
TRXProtoQueues* shared_rx_queue;
std::vector<mqd_t> ipc_rx_queues;
mqd_t ipc_tx_queue;


// One thread works full time driving the XBee
void xbee_driver_thread(int id) {
  station_main_loop(shared_tx_queue, shared_rx_queue);
}

// One thread works full time managing IPC communication
void dispatching_thread(int id) {
  dispatcher_main_loop(*shared_tx_queue, *shared_rx_queue, ipc_tx_queue, ipc_rx_queues);
}

int main() {
  // We have cross-thread communication
  shared_tx_queue = new TRXProtoQueues(MAX_QUEUE_SIZE);
  shared_rx_queue = new TRXProtoQueues(MAX_QUEUE_SIZE);

  // And we have cross-process communication. This process will handle open/close
  ipc_tx_queue = StationIPC::open_queue(StationIPC::TX_QUEUE);
  ipc_rx_queues = StationIPC::get_rx_subsribers_qids();
  
  // Wait for worker threads to finish
  std::thread xbee_driver(xbee_driver_thread, 1);
  std::thread dispatcher(dispatching_thread, 2);
  xbee_driver.join();
  dispatcher.join();

  // Clean up shared datastructures
  delete shared_tx_queue;
  delete shared_rx_queue;

  // Close the message queues (local process consequences)
  StationIPC::close_queue(ipc_tx_queue);
  for (auto qid : ipc_rx_queues) {
    StationIPC::close_queue(qid);
  }

  // Unlink the message queues (system wide consequences)
  StationIPC::unlink_queue(StationIPC::TX_QUEUE);
  StationIPC::unlink_queue(StationIPC::RX_QUEUE_SIMULATOR);
  return 0;
}
