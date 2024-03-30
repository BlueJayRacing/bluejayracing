#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>


#include "ipc_config.h"
#include "baja_live_comm.pb.h"

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  // Open the message queue, return if it fails
  mqd_t rx_queue = BajaIPC::open_queue(CarIPC::ADC_DRIVER_TO_BROKER_QUEUE, false);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  while (true) {
    usleep(1000000);
    // Let's send a message!
    std::cout << "Sending message!" << std::endl;
    std::string payload = "hello";
    int err = BajaIPC::send_message(rx_queue, payload);
    std::cout << "Sent message!" << std::endl;
}
}
