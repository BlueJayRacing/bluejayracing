#include "ipc/queue_manage.h"
#include "xbee/xbee_baja_network_config.h"
#include "ipc/trx_queues.h"
#include "baja_live_comm.pb.h"

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues *tx_queues)
{
  // Return the combined number of payloads available in all queues
  return tx_queues->get_total_size();
}

// Build up to a max size LiveCOmm objjec out of data from a vector of fields
template <typename T>
LiveComm build_message(std::vector<TRXProtoQueues::FieldID> field_ids, TRXProtoQueues *tx_queues)
{
  // Iterate through all fields with data, iteratively building the largest possible message
  // until LiveComm.serializeToString() size exceeds XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE (this
  // constant is found in xbee_baja_network_config.h)
  int field_ids_size = field_ids.size();
  LiveComm my_live_comm = LiveComm();
  int cur_field_id = 0;
  while (tx_queues->get_total_size() > 0)
  {
    if (tx_queues->template get_size<T>(cur_field_id) <= 0)
    {
      cur_field_id = (cur_field_id + 1) % field_ids.size();
      break;
    }
    LiveComm test_livecomm = test_add_data(my_live_comm, cur_field_id, tx_queues);
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
}

LiveComm test_add_data(LiveComm msg, TRXProtoQueue::FieldID field_id, TRXProtoQueue tx_queues)
{
  tx_queues->template peek<T>(field_ids[cur_field_id]);
  switch (field_id)
  {
  case TRXProtoQueues::GPS_QUEUE_ID:
    msg.add_gps(data);
    break;

  case TRXProtoQueues::LOCALIZATION_QUEUE_ID:
    msg.add_localization(data);
    break;

  case TRXProtoQueues::COMMUNICATION_QUEUE_ID:
    msg.add_communication(data);
    break;

  case TRXProtoQueues::TIMESTAMP_QUEUE_ID:
    msg.add_timestamp(data);
    break;

  case TRXProtoQueues::ANALOG_CH_QUEUE_ID:
    msg.add_analog_ch(data);
    break;

  case TRXProtoQueues::CAR_STATE_QUEUE_ID:
    msg.add_car_state(data);
    break;
  }
  return msg;
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
  // rx_queues->enqueue(my_gps, TRXProtoQueues::FieldID::GPS_QUEUE_ID); // Eg on how to enqueue a recieved GPS object
  return 0;
}