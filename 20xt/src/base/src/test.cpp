#include <iostream>
#include <mqueue.h>
#include <unistd.h>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"

// An infinite loop which attempts to read bytes from an ipc queue
int main()
{
  int qid = StationIPC::open_queue(StationIPC::TX_QUEUE);
  if (qid == -1)
  {
    std::cout << "Failed to get queue ID" << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    return EXIT_FAILURE;
  }

  char buffer[StationIPC::MAX_MSG_SIZE];
  ssize_t err = mq_receive(qid, buffer, StationIPC::MAX_MSG_SIZE, NULL);

  while (true) {
    err = mq_receive(qid, buffer, StationIPC::MAX_MSG_SIZE, NULL);
    if (err == -1 && errno == EAGAIN) {
      usleep(100000);
      std::cout << "No message available..." << std::endl;
      continue;
    } else if (err == -1) {
      std::cout << "Could not read message from queue. Errno " << errno << std::endl;
      errno = 0;
      return EXIT_FAILURE;
    }

    std::cout << "Read " << err << " bytes from queue" << std::endl;
    Observation observation;
    observation.ParseFromArray(buffer, err);
    std::cout << "Debug string: " << observation.DebugString() << std::endl;
  }
  return EXIT_SUCCESS;
}