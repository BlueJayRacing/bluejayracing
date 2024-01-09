#ifndef XBEE_CONNECTION_H
#define XBEE_CONNECTION_H

#include <string>
#include "connection.h"

class XBeeConnection : public Connection {
public:
  XBeeConnection();
  ~XBeeConnection();

  Status open() override;
  bool is_open() const override;
  void close() override;

  Status tx_status() const override;
  Status send(const std::string msg) override;

  Status tick() override;
  int num_messages_available() const override;
  std::string pop_message() override;
};

#endif // XBEE_CONNECTION_H