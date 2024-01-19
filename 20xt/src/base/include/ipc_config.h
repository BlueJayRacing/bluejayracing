#ifndef STATION_IPC_CONFIG_H
#define STATION_IPC_CONFIG_H

#include <vector>
#include <map>
#include <string>
#include <mqueue.h>

namespace StationIPC {
  const uint32_t MAX_MSG_SIZE = 1024;
  const std::string TX_QID_FNAME = "/remote_comm_tx_queue";
  const std::string RX_QID_FOR_SIMULATOR_FNAME = "/remote_comm_rx_queue_for_simulator";
  const std::string RX_QID_FOR_LOGGER_FNAME = "/remote_comm_rx_queue_for_logger";

  const mqd_t get_mqd(std::string q_fname) {
    // Hmm, we have to set non-block here
    return mq_open(q_fname.c_str(), O_CREAT | O_RDWR, 0644, NULL);
  }

  const std::vector<int> get_rx_subsribers_qids() {
    std::vector<int> qids = {
      get_mqd(RX_QID_FOR_SIMULATOR_FNAME),
      get_mqd(RX_QID_FOR_LOGGER_FNAME),
    };
    return qids;
  }

}


#endif