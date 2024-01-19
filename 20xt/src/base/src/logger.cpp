#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include "ipc_config.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main() {
  int qid = StationIPC::get_mqd(StationIPC::RX_QID_FOR_LOGGER_FNAME);
  if (qid == -1) {
    std::cout << "Failed to get queue ID" << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  char data[100];

  int err = mq_receive
  if (err == -1) {
    std::cout << "Could not read message from queue #" << qid << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Read data: '" << data << "'" << std::endl; 
  return EXIT_SUCCESS;
}