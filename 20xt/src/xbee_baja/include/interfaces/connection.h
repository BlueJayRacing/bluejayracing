#ifndef REMOTE_CONNECTION_H
#define REMOTE_CONNECTION_H

#include <string>

class Connection {
public:
  enum Status {
    SUCCESS,            // connection was successful
    RECOVERABLE_ERROR,    // connection failed, try again
    IRRECOVERABLE_ERROR,// connection failed, do not try again
    ALREADY_OPEN,       // connection is already open
    MSG_TOO_LARGE,       // Message is too large
    QUEUE_FULL,          // send queue is full
    SEND_FAILED,         // unkown, recoverable error occured
    MSGS_RECIEVED,      // messages are available for consumption
    NO_MESSAGES_RX,        // no messages available
  };
  
  virtual Status open() = 0;
  virtual bool is_open() const = 0;
  virtual void close() = 0;

  virtual int num_msgs_queued_for_tx() = 0;
  virtual Status send(const std::string msg) = 0;

  virtual Status tick() = 0;
  virtual int num_messages_available() const = 0;
  virtual std::string pop_message() = 0;
};

#endif // REMOTE_CONNECTION_H