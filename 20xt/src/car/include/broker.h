#ifndef CAR_BAJA_BROKER_H
#define CAR_BAJA_BROKER_H

#include <mqueue.h>
#include <vector>

void try_broker_data(const std::vector<mqd_t>& producer_queues, const std::vector<mqd_t> &subscriber_queues);

#endif
