#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"
#include "adc.hpp"

#define NUMADC 4

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  int fd2 = open("/dev/i2c-6", O_RDWR);

  ADC adcs[NUMADC];

  for (int i = 0; i < NUMADC; i++) {
    adcs[i] = ADC(fd2, i, false);
  }

  // Open the message queue, return if it fails
  mqd_t rx_queue = BajaIPC::open_queue(CarIPC::ADC_DRIVER_TO_BROKER_QUEUE, false);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  while (true) {
    usleep(1000000);
    // Let's send a message!

    for (int i = 0; i < NUMADC; i++) {
    	std::vector<double> data = adcs[i].read();
        std::cout << data[0] << std::endl;
    }
}
}
