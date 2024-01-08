#ifndef XBEE_CONNECTION_H
#define XBEE_CONNECTION_H

#include <string>
#include "connection.h"

class XBeeConnection : public Connection {
public:
    XBeeConnection();
    ~XBeeConnection();

    int open() override;
    bool is_open() const override;
    void close() override;

    SendResult tx_status() const override;
    SendResult send(const std::string msg) override;
    RecieveStatus rx_status() override;
    std::string get_message() override;

    // TODO: not implemented, this is just scaffolding
};

#endif // XBEE_CONNECTION_H