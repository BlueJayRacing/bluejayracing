#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "broker.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"
#include "proto_helpers.h"

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int main()
{
  std::cout << "starting IPC queue broker" << std::endl;

  // Open the queues
  const std::vector<mqd_t> producer_queues = {
    BajaIPC::open_queue(CarIPC::MQTT_CLIENT_TO_BROKER_QUEUE, false),
    BajaIPC::open_queue(CarIPC::ADC_DRIVER_TO_BROKER_QUEUE, false),
    BajaIPC::open_queue(CarIPC::BROKER_TO_SD_WRITER_QUEUE, false),
  };
  for (int i = 0; i < producer_queues.size(); i++) {
    if (producer_queues[i] == -1) {
      std::cout << "Failed to get producer queue #" << i << " Errno " << errno << std::endl;
      return EXIT_FAILURE;
    }
  }

  // Subscribed Queues
  const mqd_t to_transmit_queue = BajaIPC::open_queue(CarIPC::BROKER_TO_TRANSMIT_PRIORITIZER_QUEUE, false);
  if (to_transmit_queue == -1) {
    std::cout << "Failed to get broker-to-transmit-prioritizer queue" << " Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  const mqd_t sd_writer_queue = BajaIPC::open_queue(CarIPC::BROKER_TO_SD_WRITER_QUEUE, false);
  if (sd_writer_queue == -1) {
    std::cout << "Failed to get broker-to-sd-writer queue" << " Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }
  
  // Main loop
  std::cout << "entering main loop..." << std::endl;
  while (true) {
    try_broker_data(producer_queues, to_transmit_queue, sd_writer_queue);
  }
  return 0;
}

/* Check the producer queues, copy and dispatch to subscribers */
void try_broker_data(const std::vector<mqd_t>& producer_queues, const mqd_t to_transmit_queue, const mqd_t sd_writer_queue)
{
  for (mqd_t producer_qid : producer_queues) {
    std::string msg = BajaIPC::get_message(producer_qid);
    if (msg == "") {
      continue; // Empty Queue
    }

    // The radio can't handle all of it anyways, TODO: Should we subsample to increase performance?
    BajaIPC::send_message(to_transmit_queue, msg);
    BajaIPC::send_message(sd_writer_queue, msg);
  }
}