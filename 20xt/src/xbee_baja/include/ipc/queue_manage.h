#ifndef QUEUE_FUNCS
#define QUEUE_FUNCS

#include "baja_live_comm.pb.h"
#include "ipc/trx_queues.h"

// Adapter functions allowing the xbee-driver loop to easily
// manage the TRX_QUEUES with native data. Parse into Observation objects
// or parsing data from Observation objects and putting that data 
// into the queues.

// Minimum number of payloads available overall
int num_payloads_available(TRXProtoQueues* tx_queues);

// Min number of payloads available for a given field
int num_payloads_available(int field_id);

// Build up to a max size Observation object out of data from all fields
LiveComm build_message(TRXProtoQueues* queues);

// Build up to a max size Observation objject out of data from a vector of fields
LiveComm build_message(std::vector<int> field_ids, TRXProtoQueues* tx_queues);

// Decompose and distribute to the appropriate queues
int distribute_message(Observation msg, TRXProtoQueues* queues);

Observation _test_add_data(Observation dest, Observation src, int field_id);
#endif // QUEUE_FUNCS