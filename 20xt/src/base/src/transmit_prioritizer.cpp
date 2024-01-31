#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "mains/transmit_prioritizer.h"
#include "baja_live_comm.pb.h"
#include "helpers/ipc_config.h"
#include "helpers/live_comm_factory.h"

static Observation* remainder_data = nullptr;

// Serve as a distributer between the TRXProtoQueues and the POSIX mqueues
int main()
{
  // Open queues
  const mqd_t radio_queue = StationIPC::open_queue(StationIPC::XBEE_DRIVER_TO_TX_QUEUE);
  if (radio_queue == -1) {
    std::cout << "Failed to get radio queue. Errno " << errno << std::endl;
    return EXIT_FAILURE;
  }

  const std::vector<mqd_t> data_queues = {
    StationIPC::open_queue(StationIPC::RTK_CORRECTOR_TX_QUEUE),
    StationIPC::open_queue(StationIPC::PIT_COMMANDS_TX_QUEUE),
  };
  for (int i = 0; i < data_queues.size(); i++) {
    if (data_queues[i] == -1) {
      std::cout << "Failed to get data queue #" << i << " Errno " << errno << std::endl;
      return EXIT_FAILURE;
    }
  }

  // We're using POSIX queues, so sending a message is NOT STATELESS
  while (true) {
    usleep(100000);
    std::cout << "transmit prioritizer running..." << std::endl;
    std::string msg = build_message(data_queues);
    if (msg.empty()) {
      continue;
    }

    // DEBUG
    std::cout << "built and enqueuing message of size " << msg.size() << std::endl;
    LiveComm live_comm;
    live_comm.ParseFromString(msg);
    std::cout << live_comm.DebugString() << std::endl;
    // END DEBUG

    int result = StationIPC::send_message(radio_queue, msg);
    if (result == StationIPC::QUEUE_FULL) {
      StationIPC::get_message(radio_queue);
      result = StationIPC::send_message(radio_queue, msg);
    }
    if (result == StationIPC::SEND_ERROR) {
      std::cerr << "Could not enqeue message" << std::endl;
    }
  }

  return EXIT_SUCCESS;
}

// Prioritization logic can go in here
std::string build_message(const std::vector<mqd_t>& data_queues)
{
  LiveCommFactory factory = LiveCommFactory();
  std::string valid_msg = "";

  // One entry, we should consume the left over data
  if (remainder_data != nullptr) {
    factory.add_observation(*remainder_data);
    remainder_data = nullptr;
    valid_msg = factory.get_live_comm().SerializeAsString();
  }

  // Keep adding to the message
  int previous_queue = 0;
  while (true) {
    // Collect next data point
    Observation data;
    previous_queue = get_next_data(&data, previous_queue, data_queues);
    if (previous_queue == -1) {
      break;
    }

    // Attemp to pack the next data point
    factory.add_observation(data);
    std::string test_message = factory.get_live_comm().SerializeAsString();
    if (!is_valid_radio_message(test_message))
    {
      remainder_data->CopyFrom(data);
      break; // Could not use dequed data, save for next time
    }

    // This message is valid!
    valid_msg = test_message;
  }
  return valid_msg;
}

// Retrieve an Observation data from the tx queues, if any data available in any queue. Start
// from the queue after the one which was last used. Return -1 if no data available
int get_next_data(Observation* data_buffer, const int prev_index, const std::vector<mqd_t> &tx_queues) {
  // We need to try every queue again, with previous index as lowest priority
  for (int i = 1; i <= tx_queues.size(); i++){
    int queue_index = (i + prev_index) % tx_queues.size();
    std::string data = StationIPC::get_message(tx_queues[queue_index]);
    if (data.empty()) {
      continue;
    }

    data_buffer->ParseFromString(data);
    return queue_index;
  }
  return -1;
}

bool is_valid_radio_message(std::string msg)
{  
  // Must not be an empty message
  if (msg.length() == 0 || msg.length() > StationIPC::MAX_RADIO_MSG_SIZE)
  {
    return false;
  }
  return true;
}