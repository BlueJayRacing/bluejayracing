#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "mains/transmit_prioritizer.h"
#include "baja_live_comm.pb.h"
#include "helpers/ipc_config.h"
#include "helpers/live_comm_factory.h"

// Use this process to mock the transmit prioritizer program. Always
// sends 100 byte messages to the radio queue. Intended to stress test
// the XBee

static const std::string TEST_MESSAGE = "This is a 100 byte string yk. It is like writing an essay and trying to get the exact 100 word count";

int main()
{
  std::cout << "starting stress test prioritzer" << std::endl;
  
  // Open queues
  const mqd_t radio_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_TO_TX_QUEUE);
  if (radio_queue == -1) {
    std::cout << "Failed to get radio queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // We're using POSIX queues, so sending a message is NOT STATELESS
  while (true) {
    int result = StationIPC::send_message(radio_queue, TEST_MESSAGE);
    if (result == StationIPC::QUEUE_FULL) {
      StationIPC::get_message(radio_queue);
      result = StationIPC::send_message(radio_queue, TEST_MESSAGE);
    }
    if (result == StationIPC::SEND_ERROR) {
      std::cerr << "Could not enqeue message" << std::endl;
    }
  }

  return EXIT_SUCCESS;
}