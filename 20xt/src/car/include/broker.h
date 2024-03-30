#ifndef CAR_BAJA_BROKER_H
#define CAR_BAJA_BROKER_H

#include <mqueue.h>
#include <vector>

void try_broker_data(const std::vector<mqd_t>& producer_queues, const mqd_t to_transmit_queue, const mqd_t sd_writer_queue);

#endif
