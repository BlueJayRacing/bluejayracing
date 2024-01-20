#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

#include "ipc_config.h"

// Produce a single message to the LOGGER queue
int main() {
  mqd_t qid = StationIPC::get_message_queue_des(StationIPC::RX_QUEUE_LOGGER);
  if (qid == -1) {
    std::cout << "Failed to get queue ID" << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  char message[] = "My message is variable length";
  if (sizeof(message) > StationIPC::MAX_MSG_SIZE) {
    std::cout << "Message is too long" << std::endl;
    return EXIT_FAILURE;
  }

  int err = mq_send(qid, message, sizeof(message), 0);
  if (err == -1) {
    std::cout << "Could not send the message" << std::endl;
    if (errno == EAGAIN) {
      std::cout << "Queue is full (sending thread did not wait for empty queue)" << std::endl;
    } else {
      std::cout << "Errno: " << errno << std::endl;
    }
    return EXIT_FAILURE;
  }

  std::cout << "Sent data to queue #" << qid << "" << std::endl; 
  return EXIT_SUCCESS;
}