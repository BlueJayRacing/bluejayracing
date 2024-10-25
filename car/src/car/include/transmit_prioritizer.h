#ifndef CAR_BAJA_TRANSMIT_PRIORITIZER
#define CAR_BAJA_TRANSMIT_PRIORITIZER

#include "proto/baja_live_comm.pb.h"
#include <mqueue.h>

std::string build_message(const mqd_t queue_from_broker);
Observation get_next_data(const mqd_t queue_from_broker);
bool is_valid_radio_message(std::string msg);

#endif