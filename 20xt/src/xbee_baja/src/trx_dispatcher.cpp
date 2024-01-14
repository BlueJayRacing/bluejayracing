#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "mains/trx_dispatcher.h"
#include "ipc/trx_queues.h"
#include "baja_live_comm.pb.h"

int dispatcher_main_loop(TRXProtoQueues& shared_tx_queue, TRXProtoQueues& shared_rx_queue,
                         const int ipc_tx_queue, const std::vector<int> &ipc_rx_queues)
{
  return 0;
}