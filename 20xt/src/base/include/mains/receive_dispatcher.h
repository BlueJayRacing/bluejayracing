#ifndef RECEIVE_DISPATCHER_H
#define RECEIVE_DISPATCHER_H

#include <mqueue.h>
#include <vector>

void try_dispatch_recieved_data(const mqd_t& radio_rx_queue, const std::vector<mqd_t>& ipc_rx_queues);

#endif