#ifndef STATION_IPC_CONFIG_H
#define STATION_IPC_CONFIG_H

#include <vector>
#include <string>
#include <mqueue.h>

namespace StationIPC {
  const uint32_t MAX_QUEUE_MSG_SIZE = 1024;
  const uint32_t MAX_RADIO_MSG_SIZE = 100;

  // Queues of LiveComm from the car or bound to the car
  const std::string XBEE_DRIVER_TO_TX_QUEUE = "/xbee_driver_tx_queue";
  const std::string XBEE_DRIVER_RX_QUEUE = "/xbee_driver_rx_queue";

  // Queues of Observation data to be transmitted to car
  const std::string RTK_CORRECTOR_TX_QUEUE = "/rtk_corrector_tx_queue";
  const std::string PIT_COMMANDS_TX_QUEUE = "/pit_commands_tx_queue";

  // Queues of Observation data recieved from the car
  const std::string LOGGER_RX_QUEUE = "/logger_rx_queue";
  const std::string SIMULATION_RX_QUEUE = "/simulation_rx_queue";


  const int QUEUE_MODE = 0644;
  const int QUEUE_FLAGS = O_CREAT | O_RDWR | O_NONBLOCK;
  const mq_attr QUEUE_ATTRIBUTES = {
    .mq_flags = 0,
    .mq_maxmsg = 10,
    .mq_msgsize = MAX_QUEUE_MSG_SIZE,
    .mq_curmsgs = 0,
  };

  // Return codes detailed in POSIX message queues
  const mqd_t open_queue(std::string q_fname);
  const int close_queue(mqd_t qid);
  const int unlink_queue(std::string q_fname);

  // Get a message from the queue, return empty string if no message. Non blocking
  const std::string get_message(mqd_t qid);

  // Send a message to the queue
  enum SendResult {
    SUCCESS,
    QUEUE_FULL,
    SEND_ERROR,
  };
  const int send_message(mqd_t qid, const std::string& msg);
}

#endif