#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "baja_live_comm.pb.h"
#include "ipc_config.h"

static const std::string TEST_MESSAGE =
    "This is a 100 byte string yk. It is like writing an essay and trying to get the exact 100 word count";

/* This process mocks the transmit_prioritizer program. Always sends 100 byte messages
  to the radio transmit queue. Intended to be used to stress test the Xbee */
int main()
{
    std::cout << "starting stress test prioritzer" << std::endl;

    // Open queues
    const mqd_t radio_queue = BajaIPC::open_queue(StationIPC::PRIORITIZER_TO_XBEE_DRIVER, true);
    if (radio_queue == -1) {
        std::cout << "Failed to get radio queue. Errno " << errno << std::endl;
        return EXIT_FAILURE;
    }

    int i = 0;
    while (true) {
        int result = BajaIPC::send_message(radio_queue, TEST_MESSAGE); // Blocking
        if (result == BajaIPC::SEND_ERROR) {
            std::cerr << "Could not enqeue message" << std::endl;
        }
        i++;
        std::cout << "Sent message " << i << std::endl;
    }

    return EXIT_SUCCESS;
}