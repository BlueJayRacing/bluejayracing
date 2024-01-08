#ifndef REMOTE_CONNECTION_H
#define REMOTE_CONNECTION_H

#include <string>

class Connection {
public:
  virtual void open() = 0;
  virtual bool is_open() const = 0;
  virtual void close() = 0;

  enum SendResult {
    SUCCESS,             // send was successful
    MSG_TOO_LARGE,       // Message is too large
    QUEUE_FULL,          // send queue is full
    SEND_FAILED,         // unkown, recoverable error occured
    CONNECTION_BROKEN,   // irrecoverable error occured
  };
  virtual SendResult tx_status() const = 0;
  virtual SendResult send(const std::string msg) = 0;

  enum RecieveStatus {
    MSG_AVAILABLE,      // messages are available for consumption
    NO_MESSAGES,        // no messages available
    CONNECTION_BROKEN,  // an irrecoverable error occured in connection
  };
  virtual RecieveStatus rx_status() = 0;
  virtual std::string get_message() = 0;
};

#endif // REMOTE_CONNECTION_H