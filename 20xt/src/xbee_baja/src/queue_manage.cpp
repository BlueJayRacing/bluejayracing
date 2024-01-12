#include "ipc/queue_manage.h"

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues* tx_queues) {
  // Return the combined number of payloads available in all queues
  return 1; 
}

// Build up to a max size LiveCOmm objjec out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids, TRXProtoQueues* tx_queues) {
  // Iterate through all fields with data, iteratively building the largest possible message
  // until LiveComm.serializeToString() size exceeds XbeeBajaNetworkConfig::MAX_PAYLOAD_SIZE (this
  // constant is found in xbee_baja_network_config.h)

  GPS my_gps = tx_queues->dequeue(TRXProtoQueues::FieldID::GPS_QUEUE_ID); // Eg on how to dequeue a GPS object
  GPS my_gps = tx_queues->peek(TRXProtoQueues::FieldID::GPS_QUEUE_ID); // Eg on how to peek the front of a queue

}

// Build up to a max size LiveComm object out of data from all fields
LiveComm build_message(TRXProtoQueues* tx_queues) {
  // Call build_message(fields_ids, tx_queues), passing in field_id vector
  // with all of the field IDs
  return LiveComm();
}

// Decompose and distribute to the appropriate queues
int distribute_message(LiveComm msg, TRXProtoQueues* rx_queues) {
  // Decompose the LiveComm message into its fields, and enqueue
  // that data into the queue with appropriate field ID

  GPS my_gps = GPS();
  rx_queues->enqueue(my_gps, TRXProtoQueues::FieldID::GPS_QUEUE_ID); // Eg on how to enqueue a recieved GPS object
  return 0;
}