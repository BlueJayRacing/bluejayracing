#include "frame_handling.h"
#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee_connection.h"

int xbee_tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *context) {
  return EXIT_SUCCESS;
}

int xbee_receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *context) {
  return EXIT_SUCCESS;
}