#ifndef XBEE_CONNECTION_H
#define XBEE_CONNECTION_H

#include <string>
#include "connection.h"

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

private:
  bool is_open;
};

#endif // XBEE_CONNECTION_H