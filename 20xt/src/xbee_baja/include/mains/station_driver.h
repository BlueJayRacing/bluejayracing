#ifndef STATION_DRIVER_H
#define STATION_DRIVER_H

#include "interfaces/connection.h"
#include "crossthread/trx_queues.h"

static const int MAX_SEND_RETRIES = 3;

int station_main_loop(TRXProtoQueues* tx_queues, ObservationQueue* rx_queue);
int _try_transmit_data(Connection* conn, TRXProtoQueues* tx_queues);
int _try_recieve_data(Connection* conn, ObservationQueue* rx_queue);

#endif // STATION_DRIVER_H