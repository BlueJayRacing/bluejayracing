#include <iostream>
#include <mqueue.h>
#include <vector>
#include <unistd.h>

#include "mains/pit_commands.h"
#include "helpers/ipc_config.h"
#include "baja_live_comm.pb.h"

std::string read_raw_rtcm() {
  // TODO: return the exact string of bytes which need to be re-emitted into the
  // GPS on the car
  return "My string of bytes";
}

Observation build_rtk_correction() {
  std::string rtcm_correction read_raw_rtcm();
  
  RTKCorrection* correction = new RTKCorrection();
  correction->set_rtk_correction(rtcm_correction);

  Observation obs;
  obs.set_allocated_rtk_correction(correction);
  return obs;
}

// We want to produce a series of dummy Communcation values to the TX queue
int main () {
  // Open the message queue, return if it fails
  mqd_t tx_queue = StationIPC::open_queue(StationIPC::RTK_CORRECTOR_TX_QUEUE, false);
  if (tx_queue == -1) {
    std::cout << "Failed to get transmit queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  while (true) {
    // Should be a blocking call which retrieves an Observation with an RTK correction
    std::string payload = build_rtk_correction().serialize_as_string();

    int err = StationIPC::send_message(tx_queue, payload);
    if (err == StationIPC::QUEUE_FULL) {
      std::cout << "Queue is full, dequeing before enqueing" << std::endl;
      StationIPC::get_message(tx_queue);
      err = StationIPC::send_message(tx_queue, payload);
    }

    if (err == StationIPC::SEND_ERROR) {
      std::cerr << "Could not send RTK correction" << std::endl;
    }
  }
}