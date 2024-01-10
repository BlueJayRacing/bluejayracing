#ifndef FRAME_HANDLING_H
#define FRAME_HANDLING_H

#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee_connection.h"

const XBeeConnection* global_xbee_conn_obj;

// Add more status handler functions in the same fashion
int tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *context);

int receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *context);

// Your code here
const xbee_dispatch_table_entry_t xbee_frame_handlers[] = {
  {XBEE_FRAME_TRANSMIT_STATUS, 0, tx_status_handler, (void *) global_xbee_conn_obj},
  {XBEE_FRAME_RECEIVE_EXPLICIT, 0, receive_handler, (void *) global_xbee_conn_obj},
  {XBEE_FRAME_RECEIVE, 0, receive_handler, (void *) global_xbee_conn_obj},
  XBEE_FRAME_HANDLE_LOCAL_AT,
  XBEE_FRAME_TABLE_END};

#endif // FRAME_HANDLING_H
