#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>


#include "mains/pit_commands.h"
#include "helpers/ipc_config.h"
#include "baja_live_comm.pb.h"

Observation get_dummy_observation() {
  Communication* comm = new Communication();
  comm->set_instruction(Communication_DriverInstruction::Communication_DriverInstruction_STOP_FOR_PIT);
  Observation obs;
  obs.set_allocated_communication(comm);
  return obs;
}

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  // Open the message queue, return if it fails
  mqd_t rx_queue = StationIPC::open_queue(StationIPC::PIT_COMMANDS_TX_QUEUE);
  if (rx_queue == -1) {
    std::cout << "Failed to get recieve queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // Get the dummy data
  const int NUM_OBSERVATIONS = 10;
  std::vector<Observation> observations = get_dummy_observations(NUM_OBSERVATIONS);

  for (true) {
    usleep(10000)
    // Let's send a message!
    std::string payload = get_dummy_observation().SerializeAsString();
    int err = StationIPC::send_message(rx_queue, payload);
    if (err == StationIPC::QUEUE_FULL) {
      std::cout << "Queue is full, dequeing before enqueing" << std::endl;
      StationIPC::get_message(rx_queue);
      err = StationIPC::send_message(rx_queue, payload);
    }

    if (err == StationIPC::SEND_ERROR) {
      std::cerr << "Could not send the message" << std::endl;
    }
    std::cout << "Sent pit command to IPC queue" << std::endl;
  }
}