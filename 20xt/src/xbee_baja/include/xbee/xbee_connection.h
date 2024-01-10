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

  Status tx_status() const override; // Status of last transmission/connection
  Status send(const std::string msg) override;

  Status tick() override;
  int num_messages_available() const override;
  std::string pop_message() override;

  static std::string what_is_this_class();

private:
  bool conn_open;
  bool send_succeeded;
  std::queue<std::string>* rx_queue;

  xbee_serial_t serial;
  xbee_dev_t xbee;

  // Xbee Frame Handlers
  static int tx_status_handler(xbee_dev_t *xbee, const void FAR *raw, 
                      uint16_t length, void FAR *conn_context);

  static int receive_handler(xbee_dev_t *xbee, const void FAR *raw, 
                        uint16_t length, void FAR *conn_context);
};

#endif // XBEE_CONNECTION_H


