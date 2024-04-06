#ifndef XBEE_DRIVER_H
#define XBEE_DRIVER_H

#include "interfaces/connection.h"

int try_transmit_data(Connection* conn, const mqd_t tx_queue);
int try_recieve_data(Connection* conn, const mqd_t rx_queue);

#endif