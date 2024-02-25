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
  XBeeConnection(const std::string serial_device, const int baudrate, const int cwnd_size);
  ~XBeeConnection();

  Status open() override; // Open a connection or return failure
  bool is_open() const override; // Check if this connection object is active/open
  void close() override; // Disconnect from XBee device abstraction

  Status send(const std::string msg) override; // Makes full attempt to broadcast message over radio
  virtual int num_queued_for_tx() override; // Number of transmission requests sent to Xbee that haven't been ack'd by xbee
  
  Status tick() override; // Tick the Xbee to check if any messages have been buffered
  int num_messages_available() const override; // Will not be accurate unless tick() has been called
  std::string pop_message() override; // Retrieve a single message from the post-tick buffer

private:
  const std::string serial_device;
  const int baudrate;
  bool conn_open;

  // The Xbee doesn't inform us when serial buffer is full, so
  // we are adding a congestion control window to prevent overflowing
  // the xbee serial buffer. We will allow the user to query the number
  // of outstanding messages.
  const int cwnd_size; // congestion control window
  int last_queued_frame_id;
  int last_acked_frame_id;
  
  // We want the user of this connection to be able to retrieve
  // a single message at a time, but the XBee library may return
  // multiple with a single tick. We'll store them in a queue
  std::queue<std::string>* rx_queue;

  // The Digi library will store pointers to the frame handlers
  // and serial objects, so be cautious when changing live
  xbee_serial_t serial;
  xbee_dev_t xbee;
  xbee_dispatch_table_entry_t *xbee_frame_handlers;
  Status init_baja_xbee();

  // Handle the dispatching of a received transmit status
  static int tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *conn_context);

  // Handle the dispatching of a received message. Adds the message to rx_queue
  static int receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                        uint16_t length, void FAR *conn_context);

  // Return an initialized xbee_serial_t object
  static xbee_serial_t init_serial(const std::string serial_device, const int baudrate);

  // WIP: Write the baja Xbee network settings to the xbee device
  static Status write_baja_settings(xbee_dev_t *xbee);
};

#endif // XBEE_CONNECTION_H


