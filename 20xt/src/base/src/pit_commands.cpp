#include <iostream>
#include <mqueue.h>
#include <chrono>
#include <thread>

#include "ipc_config.h"
#include "baja_live_comm.pb.h"

std::vector<Observation> get_dummy_observations(int n) {
  std::vector<Observation> observations;
  for (int i = 0; i < n; i++) {
    Communication* comm = new Communication();
    comm->set_instruction(Communication_DriverInstruction::Communication_DriverInstruction_STOP_FOR_PIT);
    Observation obs;
    obs.set_allocated_communication(comm);
    observations.push_back(obs);
  }
  return observations;
}

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  // Open the message queue, return if it fails
  mqd_t rx_queue = StationIPC::open_queue(StationIPC::TX_QUEUE);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Get the dummy data
  const int NUM_OBSERVATIONS = 10;
  std::vector<Observation> observations = get_dummy_observations(NUM_OBSERVATIONS);

  // Time to send it all. Each one is a c_string payload that's NOT null terminated
  for (int i = 0; i < NUM_OBSERVATIONS; i++) {
    
    // Let's send a message!
    std::string payload = observations[i].SerializeAsString();
    int err = StationIPC::send_message(rx_queue, payload);
    while (err == StationIPC::QUEUE_FULL) {
      std::cout << "Queue is full, waiting for empty queue" << std::endl;
      std::this_thread::sleep_for(std::chrono::microseconds(1));
      err = StationIPC::send_message(rx_queue, payload);
    }

    if (err == StationIPC::SEND_ERROR) {
      std::cout << "Could not send the message" << std::endl;
    }
  }

  std::cout << "Sent " << NUM_OBSERVATIONS << " messages to the queue" << std::endl;
}