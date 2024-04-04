#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "transmit_prioritizer.h"
#include "baja_live_comm.pb.h"
#include "ipc_config.h"
#include "live_comm_factory.h"
#include "xbee/xbee_baja_network_config.h"

static Observation* remainder_data = nullptr;

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int main()
{
  std::cout << "starting transmit prioritizer" << std::endl;
  
  // Open queues
  const mqd_t queue_from_broker = BajaIPC::open_queue(CarIPC::BROKER_TO_TRANSMIT_PRIORITIZER_QUEUE, true);
  if (queue_from_broker == -1) {
    std::cout << "Failed to get from-broker queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }
  
  const mqd_t queue_to_radio = BajaIPC::open_queue(CarIPC::PRIORITIZER_TO_XBEE_DRIVER, true);
  if (queue_to_radio == -1) {
    std::cout << "Failed to get to-radio queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  // We're using POSIX queues, so sending a message is NOT STATELESS
  while (true) {
    // TODO: Should we decrease spin rate?
    std::string msg = build_message(queue_from_broker);
    if (msg.empty()) {
      continue;
    }

    // DEBUG
    std::cout << "built and enqueuing message of size " << msg.size() << std::endl;
    LiveComm live_comm;
    live_comm.ParseFromString(msg);
    std::cout << live_comm.DebugString() << std::endl;
    // END DEBUG

    std::cout << "transmit send" << std::endl;
    int result = BajaIPC::send_message(queue_to_radio, msg);
    if (result == BajaIPC::SEND_ERROR) {
      std::cerr << "Could not enqeue message" << std::endl;
    }
  }

  return EXIT_SUCCESS;
}

/* Grab data from prioritzier and build a payload*/
std::string build_message(const mqd_t queue_from_broker)
{
  LiveComm live_comm;
  std::string valid_msg = "";

  // One entry, we should consume the left over data
  if (remainder_data != nullptr) {
    Observation* buffer = live_comm.add_observations();
    buffer->CopyFrom(*remainder_data);

    remainder_data = nullptr;
    valid_msg = live_comm.SerializeAsString();
  }

  // Keep adding to the message
  while (true) {
    // Collect next data point
    Observation data = get_next_data(queue_from_broker); // Blocking
    Observation* buffer = live_comm.add_observations();
    buffer->CopyFrom(data);

    // Test if we can pack in
    std::string test_message = live_comm.SerializeAsString();
    if (!is_valid_radio_message(test_message))
    {
      // std::cout << "message is invalid to pack" << std::endl;
      // TODO: seems to be breaking
      //remainder_data->CopyFrom(data);
      // std::cout << "done copy" << std::endl;
      break; // Could not use dequed data, save for next time
    }
    // This message is valid!
    valid_msg = test_message;
  }
  return valid_msg;
}

/* Wait for data from the broker. Blocking wait. */
Observation get_next_data(const mqd_t queue_from_broker) {
  std::string data = BajaIPC::get_message(queue_from_broker);

  Observation observation;
  observation.ParseFromString(data);
  return observation;
}

bool is_valid_radio_message(std::string msg)
{  
  // Must not be an empty message
  if (msg.length() == 0 || msg.length() > XBEE_BAJA_MAX_PAYLOAD_SIZE)
  {
    return false;
  }
  return true;
}