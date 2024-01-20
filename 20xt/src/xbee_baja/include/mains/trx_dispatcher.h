#ifndef TRX_DISPATCHER_H
#define TRX_DISPATCHER_H

#include "ipc/trx_queues.h"

int dispatcher_main_loop(TRXProtoQueues& shared_tx_queue, TRXProtoQueues& shared_rx_queue, const int ipc_tx_queue, const std::vector<int> &ipc_rx_queues);

#endif