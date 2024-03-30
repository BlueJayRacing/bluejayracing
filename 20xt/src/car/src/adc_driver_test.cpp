#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>


#include "ipc_config.h"
#include "baja_live_comm.pb.h"

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  // Open the message queue, return if it fails
  mqd_t rx_queue = BajaIPC::open_queue(CarIPC::MQTT_CLIENT_TO_AGGR_QUEUE, false);
  if (rx_queue == -1) {
    std::cout << "Failed to get queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  while (true) {
    usleep(1000000);
    // Let's receive a message!
    std::string payload = BajaIPC::get_message(rx_queue);

    if (payload == "") {
	std::cout << "Message empty!" << std::endl;
	continue;
    }

    std::cout << "Payload: " << payload << std::endl;
  }
}


