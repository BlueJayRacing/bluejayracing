#ifndef TRX_DISPATCHER_H
#define TRX_DISPATCHER_H

#include "ipc/trx_queues.h"

int dispatcher_main_loop(TRXProtoQueues* tx_queues, TRXProtoQueues* rx_queues);

#endif