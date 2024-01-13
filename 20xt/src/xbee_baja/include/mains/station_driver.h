#ifndef STATION_DRIVER_H
#define STATION_DRIVER_H

#include "interfaces/connection.h"
#include "ipc/trx_queues.h"

static const int MAX_SEND_RETRIES = 3;

int station_main_loop(TRXProtoQueues* tx_queues, TRXProtoQueues* rx_queues);
int _try_transmit_data(Connection* conn, TRXProtoQueues* tx_queues);
int _try_recieve_data(Connection* conn, TRXProtoQueues* rx_queues);

#endif // STATION_DRIVER_H