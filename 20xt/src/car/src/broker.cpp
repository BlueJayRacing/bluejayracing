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

  const std::vector<mqd_t> subscriber_queues = {
    // Add queues here...
  };
  for (int i = 0; i < subscriber_queues.size(); i++) {
    if (subscriber_queues[i] == -1) {
      std::cout << "Failed to get subscriber queue #" << i << " Errno " << errno << std::endl;
      return EXIT_FAILURE;
    }
  }
  
  // Main loop
  std::cout << "entering main loop..." << std::endl;
  while (true) {
    try_broker_data(producer_queues, subscriber_queues)
  }
  return 0;
}


/* Check the producer queues, copy and dispatch to subscribers */
void try_broker_data(const std::vector<mqd_t>& producer_queues, const std::vector<mqd_t> &subscriber_queues)
{
  for (mqd_t producer_qid : producer_queues) {
    std::string msg = BajaIPC::get_message(producer_qid);
    if (msg == "") {
      continue; // Empty Queue
    }

    // Dispatch to every subscribed proccess
    for (mqd_t subscriber_qid : subscriber_queues) {
      // As a producer, empty the queue if it's full
      int err = BajaIPC::send_message(subscriber_qid, msg);
      if (err == BajaIPC::QUEUE_FULL) {
        BajaIPC::get_message(subscriber_qid);
        BajaIPC::send_message(subscriber_qid, msg);
      }
    }
  }
}