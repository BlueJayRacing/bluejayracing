#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "mains/trx_dispatcher.h"
#include "ipc/trx_queues.h"
#include "baja_live_comm.pb.h"

int dispatcher_main_loop(TRXProtoQueues& shared_tx_queue, TRXProtoQueues& shared_rx_queue,
                         const mqd_t ipc_tx_queue, const std::vector<mqd_t> &ipc_rx_queues)
{
  return 0;
}