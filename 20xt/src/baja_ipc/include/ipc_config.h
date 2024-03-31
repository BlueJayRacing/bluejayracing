#ifndef STATION_IPC_CONFIG_H
#define STATION_IPC_CONFIG_H

#include <vector>
#include <string>
#include <mqueue.h>

namespace BajaIPC {
  const uint32_t MAX_QUEUE_MSG_SIZE = 1024;
  const uint32_t MAX_RADIO_MSG_SIZE = 100;

  const int QUEUE_MODE = 0644;
  const int QUEUE_FLAGS = O_CREAT | O_RDWR;
  const mq_attr QUEUE_ATTRIBUTES = {
    .mq_flags = 0,
    .mq_maxmsg = 10,
    .mq_msgsize = MAX_QUEUE_MSG_SIZE,
    .mq_curmsgs = 0,
  };

  // Return codes detailed in POSIX message queues
  const mqd_t open_queue(std::string q_fname, bool blocking);
  const int close_queue(mqd_t qid);
  const int unlink_queue(std::string q_fname);

  // Get a message from the queue, return empty string if no message. Non blocking
  const std::string get_message(mqd_t qid);
  const int num_messages_available(mqd_t qid);

  // Send a message to the queue
  enum SendResult {
    SUCCESS,
    QUEUE_FULL,
    SEND_ERROR,
  };
  const int send_message(mqd_t qid, const std::string& msg);
}

namespace StationIPC {
  // Queues of LiveComm from the car or bound to the car
  const std::string XBEE_DRIVER_TO_TX_QUEUE = "/xbee_driver_tx_queue";
  const std::string XBEE_DRIVER_RX_QUEUE = "/xbee_driver_rx_queue";

  // Queues of Observation data to be TRANSMITTED to car
  const std::string RTK_CORRECTOR_TX_QUEUE = "/rtk_corrector_tx_queue";
  const std::string PIT_COMMANDS_TX_QUEUE = "/pit_commands_tx_queue";

  // Queues of Observation data RECEIVED from the car
  const std::string LOGGER_RX_QUEUE = "/logger_rx_queue";
  const std::string SIMULATION_RX_QUEUE = "/simulation_rx_queue";
}

namespace CarIPC {
  // Queues of LiveComm objects recieved from the car or bound to the car
  const std::string PRIORITIZER_TO_XBEE_DRIVER = "/prioritizer_to_xbee_driver";
  const std::string XBEE_DRIVER_TO_RX_DISPATCH = "/xbee_driver_to_rx_dispatch";

  // Queues of Observation objects bound towards broker (PRODUCERS)
  const std::string MQTT_CLIENT_TO_BROKER_QUEUE = "/mqtt_client_to_broker_queue";
  const std::string ADC_DRIVER_TO_BROKER_QUEUE = "/adc_driver_to_broker_queue";

  // Queues of Observation objects coming out of broker (SUBSCRIBERS)
  const std::string BROKER_TO_SD_WRITER_QUEUE = "/broker_to_sd_writer_queue";
  const std::string BROKER_TO_TRANSMIT_PRIORITIZER_QUEUE = "/broker_to_sd_writer_queue";
}


#endif