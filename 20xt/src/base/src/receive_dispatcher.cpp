#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "mains/trx_dispatcher.h"
#include "crossthread/trx_queues.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"
#include "proto_helpers.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int main()
{
  while (true) {
    usleep(100000);
    std::cout << "Dispatcher running" << std::endl;
    try_dispatch_recieved_data(shared_rx_queue, ipc_rx_queues);
  }
  return 0;
}


// Dispatch the data recieved over radio
void try_dispatch_recieved_data(ObservationQueue& shared_rx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  if (shared_rx_queue.size() <= 0) {
    return;
  }
  
  int err;
  char buffer[StationIPC::MAX_QUEUE_MSG_SIZE];
  std::string serialized_observation = shared_rx_queue.dequeue().SerializeAsString();

  // Must dispatch to every subscribed proccess
  std::cout << "Dispatching recieved data" << std::endl;
  for (mqd_t ipc_rx_queue : ipc_rx_queues) {
    err = mq_send(ipc_rx_queue, serialized_observation.c_str(), serialized_observation.length(), 0);

    while (err == -1 && errno == EAGAIN) {
      mq_receive(ipc_rx_queue, buffer, StationIPC::MAX_MSG_SIZE, NULL); // Empty the queue!
      err = mq_send(ipc_rx_queue, serialized_observation.c_str(), serialized_observation.length(), 0);
    }
  }
}