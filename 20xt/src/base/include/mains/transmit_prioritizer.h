#ifndef TRANSMIT_PRIORITIZER_H_
#define TRANSMIT_PRIORITIZER_H_

#include <mqueue.h>
#include <vector>

#include "baja_live_comm.pb.h"
#include "helpers/live_comm_factory.h"

std::string build_message(const std::vector<mqd_t> &data_queues);
int get_next_data(Observation* data_buffer, int prev_index, const std::vector<mqd_t> &tx_queues);
bool is_valid_radio_message(std::string msg);

#endif  // TRANSMIT_PRIORITIZER_H_