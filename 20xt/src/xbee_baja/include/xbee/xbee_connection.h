#ifndef XBEE_CONNECTION_H
#define XBEE_CONNECTION_H

#include <string>
#include <queue>
#include "interfaces/connection.h"

extern "C" {
  #include "xbee/device.h"
  #include "xbee/atcmd.h"
  #include "xbee/wpan.h"
  #include "platform_config.h"
}

class XBeeConnection : public Connection {
public:
  XBeeConnection();
  ~XBeeConnection();

  Status open() override; // Initialize XBee device abstraction with Baja settings
  bool is_open() const override; // Check if this object is open
  void close() override; // Disconnect from XBee device abstraction

  Status tx_status() override; // Status of last transmission/connection
  Status send(const std::string msg) override;

  Status tick() override;
  int num_messages_available() const override;
  std::string pop_message() override;

  static std::string what_is_this_class();

private:
  bool conn_open;
  bool send_succeeded;
  
  // We want the user of this connection to be able to retrieve
  // a single message at a time, but the XBee library may return
  // multiple with a single tick. We'll store them in a queue
  std::queue<std::string>* rx_queue;
  uint8_t latest_tx_frame_id;

  // The Digi library will store pointers to the frame handlers
  // and serial objects, so be cautious when changing live
  xbee_serial_t serial;
  xbee_dev_t xbee;
  xbee_dispatch_table_entry_t *xbee_frame_handlers;
  Status init_baja_xbee();

  // Xbee Frame Handlers
  static int tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *conn_context);

  static int receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                        uint16_t length, void FAR *conn_context);

  // Device Abstraction Helpers
  static xbee_serial_t init_serial();
  static Status write_baja_settings(xbee_dev_t *xbee);
};

#endif // XBEE_CONNECTION_H


