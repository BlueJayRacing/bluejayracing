#ifndef REMOTE_CONNECTION_H
#define REMOTE_CONNECTION_H

#include <string>

class Connection {
public:
  enum ConnectionStatus {
    SUCCESS,            // connection was successful
    RECOVERABLE_ERROR,    // connection failed, try again
    IRRECOVERABLE_ERROR,// connection failed, do not try again
    ALREADY_OPEN,       // connection is already open
  };
  virtual ConnectionStatus open() = 0;
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
    MSGS_RECIEVED,      // messages are available for consumption
    NO_MESSAGES_RX,        // no messages available
    CONNECTION_BROKEN,  // an irrecoverable error occured in connection
  };
  virtual RecieveStatus tick() = 0;
  virtual int num_messages_available() const = 0;
  virtual std::string pop_message() = 0;
};

#endif // REMOTE_CONNECTION_H