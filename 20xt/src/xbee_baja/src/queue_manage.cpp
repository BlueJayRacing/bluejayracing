#include <string>

#include "crossthread/queue_manage.h"
#include "crossthread/trx_queues.h"
#include "crossthread/live_comm_factory.h"
#include "xbee/xbee_baja_network_config.h"
#include "baja_live_comm.pb.h"
#include "proto_helpers.h"

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues *tx_queues)
{
  // Return the combined number of payloads available in all queues
  return tx_queues->get_total_size();
}

bool _is_valid_message(std::string msg)
{
  // This does not check if the message can be validly parsed into a LiveComm object
  
  // Must not be an empty message
  if (msg.length() == 0)
  {
    return false;
  }
  
  // Must be of a certain length
  if (msg.length() > XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE) {
    return false;
  }

  return true;
}

// Build up to a max size Observation objjec out of data from a vector of fields
std::string build_message(std::vector<int> field_ids, TRXProtoQueues *tx_queues)
{
  LiveCommFactory factory = LiveCommFactory();
  std::string valid_msg = "";

  int i = -1;
  while (tx_queues->get_total_size() > 0)
  {
    // Skip invalid fields or fields without data
    i += (i + 1) % field_ids.size();
    if (i == field_ids.size() || tx_queues->get_size(field_ids[i]) == 0)
    {
      continue;
    }

    factory.add_observation(field_ids[i], tx_queues->front(field_ids[i]));    
    std::string test_message = factory.get_live_comm().SerializeAsString();
    
    // If test message too large, return the last valid message
    if (!_is_valid_message(test_message))
    {
      break;
    }
    tx_queues->dequeue(field_ids[i]);
    valid_msg = test_message;
  }

  std::cout << "Constructed message: " << factory.get_live_comm().DebugString() << std::endl;
  return valid_msg;
}

// Build up to a max size Observation object out of data from all fields
std::string build_message(TRXProtoQueues *tx_queues)
{
  return build_message(BajaProtoHelpers::OBSERVATION_FIELD_IDS, tx_queues);
}

// Decompose and distribute into the RX queue
int distribute_message(LiveComm msg, ObservationQueue *rx_queue)
{
  for(int i = 0; i< msg.observations_size(); i++){
    Observation obs = msg.observations(i);
    rx_queue->enqueue(obs);
  }
  return 0; // TODO: Sorry Jen, I changed this again
}