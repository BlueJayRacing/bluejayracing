#ifndef TRX_DISPATCHER_H
#define TRX_DISPATCHER_H

#include "crossthread/trx_queues.h"

int dispatcher_main_loop(TRXProtoQueues& shared_tx_queue, LiveCommQueue& shared_rx_queue, const int ipc_tx_queue, const std::vector<int> &ipc_rx_queues);

void _try_queue_data_for_transmit(TRXProtoQueues& shared_tx_queue, const mqd_t ipc_tx_queue);
void _try_dispatch_recieved_data(LiveCommQueue& shared_rx_queue, const std::vector<mqd_t> &ipc_rx_queues);

#endif