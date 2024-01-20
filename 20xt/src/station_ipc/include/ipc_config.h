#ifndef STATION_IPC_CONFIG_H
#define STATION_IPC_CONFIG_H

#include <vector>
#include <string>
#include <mqueue.h>

namespace StationIPC {
  const uint32_t MAX_MSG_SIZE = 1024;
  const std::string TX_QUEUE = "/remote_comm_tx_queue";
  const std::string RX_QUEUE_SIMULATOR = "/remote_comm_rx_queue_for_simulator";
  const std::string RX_QUEUE_LOGGER = "/remote_comm_rx_queue_for_logger";

  const int QUEUE_MODE = 0644;
  const int QUEUE_FLAGS = O_CREAT | O_RDWR | O_NONBLOCK;
  const mq_attr QUEUE_ATTRIBUTES = {
    .mq_flags = 0,
    .mq_maxmsg = 10,
    .mq_msgsize = MAX_MSG_SIZE,
    .mq_curmsgs = 0,
  };

  const mqd_t open_queue(std::string q_fname);
  const int close_queue(mqd_t qid);
  const int unlink_queue(std::string q_fname);

  // The data dispatching thread will need to access a queue for each reciever
  const std::vector<int> get_rx_subsribers_qids();
}

#endif