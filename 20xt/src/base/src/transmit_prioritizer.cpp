#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/trx_dispatcher.h"
#include "crossthread/trx_queues.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"
#include "proto_helpers.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int dispatcher_main_loop()
{
  const mqd_t ipc_radio_tx_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_TO_TX_QUEUE);
  const std::vector<mqd_t> ipc_tx_queues = {
    StationIPC::open_queue(StationIPC::RTK_CORRECTOR_TX_QUEUE),
    StationIPC::open_queue(StationIPC::PIT_COMMANDS_TX_QUEUE),
  }

  
  while (true) {
    usleep(100000);
    std::cout << "Dispatcher running" << std::endl;
    try_queue_data_for_transmit(ipc_tx_queues);
  }
  return EXIT_SUCCESS;
}

// Prioritize the data in IPC which should be trasnmitted over radio
void try_queue_data_for_transmit(const std::vector<mqd_t> &ipc_tx_queues)
{
  char buffer[StationIPC::MAX_MSG_SIZE];
  ssize_t err = mq_receive(ipc_tx_queue, buffer, StationIPC::MAX_MSG_SIZE, NULL);
  if (err== -1 && errno != EAGAIN) {
    std::cerr << "Failed reading message from TX IPC queue. Errno " << errno << std::endl;
    return;
  }
  if (err == -1 && errno == EAGAIN) {
    return; // Queue is empty :)
  }

  // TODO: Design decision to be made. Currently, only enqueue it under the first 
  // set field that we encounter. This should work since outgoing messages will
  // likely only have one field aynways. Prioritization logic will go here
  std::cout << "Queueing data from transmit" << std::endl;
  Observation observation;
  observation.ParseFromArray(buffer, err);
  std::vector<int> field_ids = BajaProtoHelpers::get_set_field_ids(observation);
  if (field_ids.size() <= 0) {
    return; // Why would a message have no data?
  }

  shared_tx_queue.enqueue(field_ids[0], observation);
}