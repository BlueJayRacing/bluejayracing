#include <iostream>
#include <mqueue.h>
#include <unistd.h>
#include <vector>

#include "mains/transmit_prioritizer.h"
#include "baja_live_comm.pb.h"
#include "helpers/ipc_config.h"
#include "helpers/live_comm_factory.h"


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

  // We're using POSIX queues, so sending a message is NOT STATELESS. Use the remainder to
  // store the data which we couldn't send in the previous iteration (but had to dequeue)
  Observation remainder;
  while (true) {
    std::cout << "Transmit prioritizer running" << std::endl;
    usleep(100000);
    std::string msg = build_message(&remainder, remainder, data_queues);

    int result = StationIPC::send_message(radio_queue, msg);
    if (result == StationIPC::QUEUE_FULL) {
      StationIPC::get_message(radio_queue);
      StationIPC::send_message(radio_queue, msg);
    }
  }

  return EXIT_SUCCESS;
}

// Prioritization logic can go in here
std::string build_message(Observation* remainder_buffer, const Observation& starter, const std::vector<mqd_t> &data_queues)
{
  // Starting message
  LiveCommFactory factory = LiveCommFactory();
  factory.add_observation(starter);
  std::string valid_msg = factory.get_live_comm().SerializeAsString();

  // Keep adding to the message
  Observation data;
  int latest_queue = 0;

  while (true) {
    latest_queue = get_next_data(&data, latest_queue, data_queues);
    if (latest_queue == -1) {
      data = Observation();
      break;
    }
    factory.add_observation(data);

    std::string test_message = factory.get_live_comm().SerializeAsString();
    if (!is_valid_radio_message(test_message))
    {
      break;
    }
    valid_msg = test_message;
  }

  remainder_buffer->CopyFrom(data);
  return valid_msg;
}

// Retrieve an Observation data from the tx queues, if any data available in any queue. Start
// from the queue after the one which was last used. Return -1 if no data available
int get_next_data(Observation* data_buffer, int prev_index, const std::vector<mqd_t> &tx_queues) {
  int i = (prev_index + 1) % tx_queues.size();

  while (i != prev_index) {
    std::string data = StationIPC::get_message(tx_queues[i]);
    if (data.empty()) {
      i = (i + 1) % tx_queues.size();
      continue;
    }

    data_buffer->ParseFromString(data);
    return i;
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