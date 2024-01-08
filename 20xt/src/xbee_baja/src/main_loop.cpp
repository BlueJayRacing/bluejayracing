#include <string>
#include <iostream>

#include "connection.h"
#include "xbee_connection.h"
#include "baja_live_comm.pb.h"
#include "trx_queues.h"

int main(void) {

  // Construct RX queues using multiqueue (create one for each field ID)
  // Construct TX queues using multiqueue (create one for each field ID)
  TRXProtoQueues tx_queues = TRXProtoQueues();
  TRXProtoQueues rx_queues = TRXProtoQueues();

  // Construct a connection, open it
  Connection* conn = new XBeeConnection();

  
  // If any of the tx queues have messages
  // ... get a payload from any queue in tx_queues
  // ... serialize it
  // ... send it

  std::vector<int> tx_fields = tx_queues.field_ids_with_data();
  for (int i = 0; i < tx_fields.size(); i++) {
    int field_id = tx_fields[i];
    LiveComm data = tx_queues.dequeue(field_id);
    
    // Needs to properly handle errors
    auto result = conn->send(data.SerializeAsString());
  }

  // If rx_status has message available
  // ... get a message
  // ... deserialize it into a LiveComm
  // ... Hand it to rx_queue, which will distribute it to the appropriate queue

  delete conn;
}