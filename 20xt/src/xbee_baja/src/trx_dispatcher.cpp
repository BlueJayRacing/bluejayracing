#include <iostream>
#include <mqueue.h>

#include "mains/trx_dispatcher.h"
#include "crossthread/trx_queues.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int dispatcher_main_loop(TRXProtoQueues& shared_tx_queue, TRXProtoQueues& shared_rx_queue,
                         const mqd_t ipc_tx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  
  // Attempt to read a message from ipc_tx_queue. Deserialize it and push it to shared_tx_queue

  // Attempt to read a message from shared_rx_queue. Serialize it and push it to all of the ipc_rx_queues
  return 0;
}