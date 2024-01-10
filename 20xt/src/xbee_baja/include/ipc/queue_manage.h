#ifndef QUEUE_FUNCS
#define QUEUE_FUNCS

#include "ipc/trx_queues.h"

// Adapter functions allowing the xbee-driver loop to easily
// manage the TRX_QUEUES with native data. Parse into LiveComm objects
// or parsing data from LiveComm objects and putting that data 
// into the queues.

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues* queues);

// Min number of payloads available for a given field
int num_payloads_available(int field_id);

// Build up to a max size LiveComm object out of data from one field
LiveComm build_message(int field_id);

// Build up to a max size LiveComm object out of data from all fields
LiveComm build_message(TRXProtoQueues* queues);

// Build up to a max size LiveCOmm objject out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids);

// Decompose and distribute to the appropriate queues
int distribute_message(LiveComm msg, TRXProtoQueues* queues);

#endif // QUEUE_FUNCS