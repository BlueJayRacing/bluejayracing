#include <iostream>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <string.h>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  const int NUM_OBSERVATIONS = 10;
  // Construct a vector of 10 Observation obejcts, each containing Communication data

  // Open the message queue, return if it fails
  mqd_t qid = StationIPC::get_message_queue_des(StationIPC::TX_QUEUE);
  if (qid == -1) {
    std::cout << "Failed to get queue ID, errno " << errno << std::endl;
    std::cout << "Check the the remote_comm process is running first" << std::endl;
    return EXIT_FAILURE;
  }

  // For 10 iterations
  //  - serialize the Obvservation into a c string
  //  - assert that the size of the string is less than IPC::MAX_MSG_SIZE (do not send null terminator)
  //  - Send the message to the queue and 
  //  - while err == -1 and errno == EAGAIN, sleep for 1 millisecond and try again
  //  - if err == -1 and errno != EAGAIN, print an error message and continue to next FOR loop iteration
}

int example_main() {
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