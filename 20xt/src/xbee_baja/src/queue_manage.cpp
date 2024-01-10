#include "ipc/queue_manage.h"

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues* queues) {
  return 0; // TODO
}

// Min number of payloads available for a given field
int num_payloads_available(int field_id) {
  return 0; // TODO
}

// Build up to a max size LiveComm object out of data from one field
LiveComm build_message(int field_id) {
  return LiveComm(); // TODO
}

// Build up to a max size LiveComm object out of data from all fields
LiveComm build_message(TRXProtoQueues* queues) {
  return LiveComm(); // TODO
}

// Build up to a max size LiveCOmm objject out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids) {
  return LiveComm(); // TODO
}

// Decompose and distribute to the appropriate queues
int distribute_message(LiveComm msg, TRXProtoQueues* queues) {
  return 0; // TODO
}