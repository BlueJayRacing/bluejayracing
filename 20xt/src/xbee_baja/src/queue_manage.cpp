#include "ipc/queue_manage.h"

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues* queues) {
  return 1; // TODO
}

// Build up to a max size LiveComm object out of data from all fields
LiveComm build_message(TRXProtoQueues* queues) {
  // TODO: This is a dummy implementation
  // GPS gps;
  // gps.set_alt(1.5);
  // gps.set_lat(2.5);
  // gps.set_long_(3.5);
  // LiveComm msg;
  // msg.set_allocated_gps(&gps);
  
  return LiveComm(); // TODO
}

// Decompose and distribute to the appropriate queues
int distribute_message(LiveComm msg, TRXProtoQueues* queues) {
  return 0; // TODO
}