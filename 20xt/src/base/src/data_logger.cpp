#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "ipc_config.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main() {
  int qid = StationIPC::open_queue(StationIPC::RX_QUEUE_LOGGER);
  if (qid == -1) {
    std::cout << "Failed to get queue ID" << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  char data[StationIPC::MAX_MSG_SIZE];

  while (true) {
    usleep(100000);
    std::cout << "Logger is running" << std::endl;
    ssize_t err = mq_receive(qid, data, StationIPC::MAX_MSG_SIZE, NULL);
    if (err == -1 && errno == EAGAIN) {
      continue;
    }
    if (err == -1 && errno != EAGAIN) {
      continue;
    }
    std::cout << "Read " << err << " bytes from queue #" << qid << std::endl;
  }
  

  int err = mq_close(qid);
  if (err == -1) {
    std::cout << "Could not close queue #" << qid << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }
  err = mq_unlink(StationIPC::RX_QUEUE_LOGGER.c_str());
  if (err == -1) {
    std::cout << "Could not unlink queue #" << qid << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Closed and unlinked queue #" << qid << std::endl;
  return EXIT_SUCCESS;
}