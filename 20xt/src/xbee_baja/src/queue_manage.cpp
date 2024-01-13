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

// Build up to a max size Observation objjec out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids, TRXProtoQueues *tx_queues)
{
  // Iterate through each field once, iteratively building the largest possible message
  // until LiveComm.serializeToString() size exceeds XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE (this
  // constant is found in xbee_baja_network_config.h)

  int field_ids_size = field_ids.size();
  Observation my_observation = Observation();
  int cur_field_id = 0;
  while (tx_queues->get_total_size() > 0)
  {
    if (tx_queues->template get_size<T>(cur_field_id) <= 0)
    {
      cur_field_id = (cur_field_id + 1) % field_ids.size();
      break;
    }
    Observation test_livecomm = test_add_data(my_observation, cur_field_id, tx_queues);
    if (test_livecomm.SerializeAsString().size() > XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE)
    {
      break;
    }
    else
    {
      my_live_comm = test_livecomm;
      tx_queues->template dequeue<T>(cur_field_id);
      cur_field_id = (cur_field_id + 1) % field_ids.size();
    }
  }
  return my_live_comm;

  return LiveComm();
}

// Copy field from src into a copy of dest. Return the copy with new data. 
// If msg already has data in that field, it will overrite with new data. 
Observation _test_add_data(Observation dest, Observation src, int field_id)
{
  const google::protobuf::FieldDescriptor *field = Observation::GetDescriptor()->FindFieldByNumber(field_id);
  const google::protobuf::Reflection *src_reflection = src.GetReflection();
  const google::protobuf::Reflection *dest_reflection = dest.GetReflection();

  // Set the dest field to a copy of the value from src_reflection
  const google::protobuf::Message& src_msg = src_reflection->GetMessage(src, field);
  google::protobuf::Message *dest_msg = dest_reflection->MutableMessage(&dest, field);
  dest_msg->CopyFrom(src_msg);
  return dest;
}

// Build up to a max size Observation object out of data from all fields
LiveComm build_message(TRXProtoQueues *tx_queues)
{
  // Call build_message(fields_ids, tx_queues), passing in field_id vector
  // with all of the field IDs
  return LiveComm();
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