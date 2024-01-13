#ifndef LIVE_COMM_QUEUE_H
#define LIVE_COMM_QUEUE_H

#include "baja_live_comm.pb.h"

class LiveCommQueue {
public:
  virtual ~LiveCommQueue() {}

  virtual bool enqueue(LiveComm data) = 0;
  virtual LiveComm front() = 0;
  virtual LiveComm dequeue() = 0;
  virtual int size() = 0;
};

#endif // LIVE_COMM_QUEUE_H
