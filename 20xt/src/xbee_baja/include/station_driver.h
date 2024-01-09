#ifndef STATION_DRIVER_H
#define STATION_DRIVER_H

#include "connection.h"
#include "trx_queues.h"

static const int MAX_SEND_RETRIES = 100;

int station_main_loop(void);
int try_produce_data(Connection* conn, TRXProtoQueues* tx_queues);
int try_consume_data(Connection* conn, TRXProtoQueues* rx_queues);

#endif // STATION_DRIVER_H