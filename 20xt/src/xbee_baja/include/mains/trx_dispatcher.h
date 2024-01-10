#ifndef TRX_DISPATCHER_H
#define TRX_DISPATCHER_H

int dispatcher_main_loop(TRXProtoQueues* tx_queues, TRXProtoQueues* rx_queues, ...);

#endif