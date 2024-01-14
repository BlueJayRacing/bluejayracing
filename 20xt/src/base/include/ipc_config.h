#ifndef STATION_IPC_CONFIG_H
#define STATION_IPC_CONFIG_H

#include <vector>
#include <map>
#include <string>
#include <sys/ipc.h>
#include <sys/msg.h>

namespace StationIPC {
  
  const std::string TX_QID_FNAME = "/etc/remote_comm_tx_queue";
  const std::string RX_QID_FOR_SIMULATOR_FNAME = "/etc/remote_comm_rx_queue_for_simulator";
  const std::string RX_QID_FOR_LOGGER_FNAME = "/etc/remote_comm_rx_queue_for_logger";

  const int get_qid(std::string q_fname) {
    return msgget(ftok(q_fname.c_str(), 'A'), 0666 | IPC_CREAT);
  }

  const std::vector<int> get_rx_subsribers_qids() {
    std::vector<int> qids = {
      get_qid(RX_QID_FOR_SIMULATOR_FNAME),
      get_qid(RX_QID_FOR_LOGGER_FNAME),
    };
    return qids;
  }

}


#endif