#include "ipc/queue_manage.h"
#include "ipc/trx_queues.h"
#include "xbee/xbee_baja_network_config.h"
#include "baja_live_comm.pb.h"

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues *tx_queues)
{
  // Return the combined number of payloads available in all queues
  return tx_queues->get_total_size();
}

bool _if_can_include_observation(const LiveComm& live_comm, const Observation& observation)
{
  // Try to place observation into an observation slot of the live_comm and
  // see if it exceeds the size limit
  LiveComm test_live_comm = live_comm;
  Observation *test_observation_slot = test_live_comm.add_observations();
  test_observation_slot->CopyFrom(observation);

  return test_live_comm.SerializeAsString().size() > XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE;
}

// Check if observation can be feasibly added to LiveComm
bool _is_valid_size(const Observation& dequeued_data) 
{
  LiveComm comm = LiveComm();
  Observation* obs_slot = comm.add_observations();
  obs_slot->CopyFrom(dequeued_data);
  return comm.SerializeAsString().size() <= XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE;
}


// Build up to a max size Observation objjec out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids, TRXProtoQueues *tx_queues)
{
  LiveComm my_live_comm = LiveComm();

  int i = 0;
  Observation my_observation = Observation();
  while (tx_queues->get_total_size() > 0) // Caution: `i` edited in loop
  {
    // If we've gone over all the fields without a size violation, we
    // need to start from first field
    if (i == field_ids.size())
    {
      Observation *observation_slot = my_live_comm.add_observations();
      observation_slot->CopyFrom(my_observation);
      my_observation.Clear();
      i = 0;
    }
    
    if (tx_queues->get_size(field_ids[i] <= 0)) {
      i += 1;
      continue;
    }

    // We need to test what happens if we add the next data point
    Observation dequeued_data = tx_queues->front(field_ids[i]);
    if (!_is_valid_size(dequeued_data)) {
      std::cerr << "ERROR: discarding too-large data point from queue #" << field_ids[i] << std::endl; 
      tx_queues->dequeue(field_ids[i]);
    }


    // If we exceed the payload size, my_live_comm is ready for transmit
    Observation test_my_observation = _test_add_data(my_observation, dequeued_data, field_ids[i]);
    if (!_if_can_include_observation(my_live_comm, test_my_observation))
    {
      Observation *observation_slot = my_live_comm.add_observations();
      observation_slot->CopyFrom(my_observation);
      break;
    }

    // We can add the data to the current observation without exceeding limit
    my_observation = test_my_observation;
    tx_queues->dequeue(field_ids[i]);
  }

  Observation *observation_slot = my_live_comm.add_observations();
  observation_slot->CopyFrom(my_observation);
  return my_live_comm;
}


// Copy field from src into a copy of dest. Return the copy with new data.
// If msg already has data in that field, it will overrite with new data.
Observation _test_add_data(Observation dest, Observation src, int field_id)
{
  const google::protobuf::FieldDescriptor *field = Observation::GetDescriptor()->FindFieldByNumber(field_id);
  const google::protobuf::Reflection *src_reflection = src.GetReflection();
  const google::protobuf::Reflection *dest_reflection = dest.GetReflection();

  // Set the dest field to a copy of the value from src_reflection
  const google::protobuf::Message &src_msg = src_reflection->GetMessage(src, field);
  google::protobuf::Message *dest_msg = dest_reflection->MutableMessage(&dest, field);
  dest_msg->CopyFrom(src_msg);
  return dest;
}

// Build up to a max size Observation object out of data from all fields
LiveComm build_message(TRXProtoQueues *tx_queues)
{
  std::vector<int> field_ids = {
      Observation::kGpsFieldNumber,
      Observation::kLocalizationFieldNumber,
      Observation::kCommunicationFieldNumber,
      Observation::kTimestampFieldNumber,
      Observation::kAnalogChFieldNumber,
      Observation::kCarStateFieldNumber,
  };

  return build_message(field_ids, tx_queues);
}

// Decompose and distribute to the appropriate queues
int distribute_message(Observation msg, TRXProtoQueues *rx_queues)
{
  // Decompose the Observation message into its fields, and enqueue
  // that data into the queue with appropriate field ID

  // GPS my_gps = GPS();
  // rx_queues->enqueue(my_gps, int); // Eg on how to enqueue a recieved GPS object
  return 0;
}