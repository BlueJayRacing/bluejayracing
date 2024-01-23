#ifndef LIVE_COMM_QUEUE_H
#define LIVE_COMM_QUEUE_H

#include "baja_live_comm.pb.h"

class ObservationQueue {
public:
  virtual ~ObservationQueue() {}

  virtual bool enqueue(Observation data) = 0;
  virtual Observation front() = 0;
  virtual Observation dequeue() = 0;
  virtual int size() = 0;
};

#endif // LIVE_COMM_QUEUE_H
