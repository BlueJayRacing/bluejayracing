#ifndef FRAME_HANDLING_H
#define FRAME_HANDLING_H

#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee_connection.h"

// Add more status handler functions in the same fashion
int xbee_tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *context);

int xbee_receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *context);

#endif // FRAME_HANDLING_H
