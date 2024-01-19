#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "ipc_config.h"

// Produce a single message to the LOGGER queue
int main() {
  mqd_t qid = StationIPC::get_mqd(StationIPC::RX_QID_FOR_LOGGER_FNAME);
  if (qid == -1) {
    std::cout << "Failed to get queue ID" << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // https://stackoverflow.com/questions/39013628/sending-variable-size-buffer-over-message-queue

  char message[] = "My message is variable length";
  int err = mq_send(qid, message, sizeof(message), 0);
  if (err == -1) {
    std::cout << "Could not send the message" << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Sent data to queue #" << qid << "" << std::endl; 
  return EXIT_SUCCESS;
}