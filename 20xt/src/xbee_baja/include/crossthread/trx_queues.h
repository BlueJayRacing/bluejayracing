#ifndef TRX_QUEUEs_H
#define TRX_QUEUEs_H

#include <map>
#include <iostream>
#include <queue>

#include "interfaces/live_comm_queue.h"
#include "crossthread/safe_live_comm_queue.h"
#include "baja_live_comm.pb.h"

static const int MAX_SIZE = 300;
// Multi queue containing all of the RX and TX data in its
// native format. Intended to be constructed by the supervisor
// thread. The supervisor thread will then pass the queues to
// the xbee-driver thread and other threads.

class TRXProtoQueues
{
public:
  TRXProtoQueues(int max_queue_size);
  ~TRXProtoQueues();

  // Get the combined size of all queues
  int get_total_size();

  // Returns the size of the queue with the given id
  int get_size(int queue_id);

  // Enqueue a Observation object which contains a payload of the specified ID
  // returns true if successful, false if the queue is full
  bool enqueue(int queue_id, Observation data);

  // Returns a Observation object which has both a payload of the ID specified
  // and (optionally) a timestamp  
  Observation front(int queue_id);

  // Returns a Observation object which has both a payload of the ID specified
  // and (optionally) a timestamp  
  Observation dequeue(int queue_id);

private:
  std::map<int, LiveCommQueue*> queues = {
      {Observation::kGpsFieldNumber, nullptr},
      {Observation::kLocalizationFieldNumber, nullptr},
      {Observation::kCommunicationFieldNumber, nullptr},
      {Observation::kCarStateFieldNumber, nullptr},
      {Observation::kAnalogChFieldNumber, nullptr},
      {Observation::kCarStateFieldNumber, nullptr}};
};

#endif // TRX_QUEUEs_H