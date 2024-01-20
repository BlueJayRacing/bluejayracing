#include <iostream>
#include <mqueue.h>

#include "mains/trx_dispatcher.h"
#include "crossthread/trx_queues.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"
#include "proto_helpers.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int dispatcher_main_loop(TRXProtoQueues& shared_tx_queue, LiveCommQueue& shared_rx_queue,
                         const mqd_t ipc_tx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  _try_queue_data_for_transmit(shared_tx_queue, ipc_tx_queue);
  _try_dispatch_recieved_data(shared_rx_queue, ipc_rx_queues);
  return 0;
}

// Prioritize the data in IPC which should be trasnmitted over radio
void _try_queue_data_for_transmit(TRXProtoQueues& shared_tx_queue, const mqd_t ipc_tx_queue)
{
  char buffer[StationIPC::MAX_MSG_SIZE];
  ssize_t err = mq_receive(ipc_tx_queue, buffer, StationIPC::MAX_MSG_SIZE, NULL);
  if (err == -1 && errno != EAGAIN) {
    std::cerr << "Failed reading message from TX IPC queue. Errno " << errno << std::endl;
    return;
  } 

  // TODO: Design decision to be made. Currently, only enqueue it under the first 
  // set field that we encounter. This should work since outgoing messages will
  // likely only have one field aynways. Prioritization logical will go here
  Observation observation;
  observation.ParseFromArray(buffer, err);
  std::vector<int> field_ids = BajaProtoHelpers::get_set_field_ids(observation);
  if (field_ids.size() > 0) {
    shared_tx_queue.enqueue(field_ids[0], observation);
  }
}


// Dispatch the data recieved over radio
void _try_dispatch_recieved_data(LiveCommQueue& shared_rx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  if (shared_rx_queue.size() <= 0) {
    return;
  }
  
  int err;
  char buffer[StationIPC::MAX_MSG_SIZE];
  std::string serialized_observation = shared_rx_queue.dequeue().SerializeAsString();

  // Must dispatch to every subscribed proccess
  for (mqd_t ipc_rx_queue : ipc_rx_queues) {
    err = mq_send(ipc_rx_queue, serialized_observation.c_str(), serialized_observation.length(), 0);

    while (err == -1 && errno != EAGAIN) {
      mq_receive(ipc_rx_queue, buffer, StationIPC::MAX_MSG_SIZE, NULL); // Empty the queue!
      err = mq_send(ipc_rx_queue, serialized_observation.c_str(), serialized_observation.length(), 0);
    }
  }
}