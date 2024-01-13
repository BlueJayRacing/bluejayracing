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

// Build up to a max size LiveCOmm objjec out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids, TRXProtoQueues *tx_queues)
{
  // Iterate through each field once, iteratively building the largest possible message
  // until LiveComm.serializeToString() size exceeds XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE (this
  // constant is found in xbee_baja_network_config.h)

  // int field_ids_size = field_ids.size();
  // LiveComm my_live_comm = LiveComm();
  // int cur_field_id = 0;
  // while (tx_queues->get_total_size() > 0)
  // {
  //   if (tx_queues->template get_size<T>(cur_field_id) <= 0)
  //   {
  //     cur_field_id = (cur_field_id + 1) % field_ids.size();
  //     break;
  //   }
  //   LiveComm test_livecomm = test_add_data(my_live_comm, cur_field_id, tx_queues);
  //   if (test_livecomm.SerializeAsString().size() > XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE)
  //   {
  //     break;
  //   }
  //   else
  //   {
  //     my_live_comm = test_livecomm;
  //     tx_queues->template dequeue<T>(cur_field_id);
  //     cur_field_id = (cur_field_id + 1) % field_ids.size();
  //   }
  // }
  // return my_live_comm;

  return LiveComm();
}

LiveComm _test_add_data(LiveComm dest, LiveComm src, int field_id)
{
  // Copy field from src into a copy of dest. Return the copy with new data. 
  // If msg already has data in that field, it will overrite with new data. 
  
  // // TODO: figure out how to make a copy of the msg object
  // LiveComm msg_copy(msg);
  
  // // Dequeue the encapsulated data from the queue
  // LiveComm new_data = tx_queues->front(field_id);
  // if (new_data == NULL) {
  //   std::cout << "ERROR: could not retrieve data from queue #" << field_id << std::endl;
  //   return msg_copy;
  // }

  // // Check that the LiveComm object has the field of interest set to non-default value
  // const google::protobuf::Reflection *reflection = new_data.GetReflection();
  // const google::protobuf::FieldDescriptor *field = new_data.GetDescriptor()->FindFieldByNumber(field_id);
  // if (field != nullptr)
  // {
  //   if (!reflection->HasField(new_data, field))
  //   {
  //     std::cout << "ERROR: LiveComm object missing field #" << queue_id << std::endl;
  //     return msg_copy;
  //   }
  // }

  // new_data has the field of interest set to non-default value, so add only that field
  // to the msg_copy object. We need to make a copy of the data, since it will have been
  // dynamically allocated.
  

  // return msg;

  return LiveComm();
}

// Build up to a max size LiveComm object out of data from all fields
LiveComm build_message(TRXProtoQueues *tx_queues)
{
  // Call build_message(fields_ids, tx_queues), passing in field_id vector
  // with all of the field IDs
  return LiveComm();
}

// Decompose and distribute to the appropriate queues
int distribute_message(LiveComm msg, TRXProtoQueues *rx_queues)
{
  // Decompose the LiveComm message into its fields, and enqueue
  // that data into the queue with appropriate field ID

  // GPS my_gps = GPS();
  // rx_queues->enqueue(my_gps, int); // Eg on how to enqueue a recieved GPS object
  return 0;
}