#ifndef TRX_QUEUEs_H
#define TRX_QUEUEs_H

#include <map>
#include <iostream>

#include "ipc/safe_queue.h"
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

  // Enqueue a LiveComm object which contains a payload of the specified ID
  // WARNING: no checking is done, you must ensure that the LiveComm object
  // has a payload of the correct ID
  void enqueue(int queue_id, LiveComm data);

  // Returns a LiveComm object which has both a payload of the ID specified
  // and (optionally) a timestamp  
  LiveComm peek(int queue_id);

  // Returns a LiveComm object which has both a payload of the ID specified
  // and (optionally) a timestamp  
  LiveComm dequeue(int queue_id);

private:
  std::map<int, SafeQueue<LiveComm> *> queues = {
      {LiveComm::kGpsFieldNumber, nullptr},
      {LiveComm::kLocalizationFieldNumber, nullptr},
      {LiveComm::kCommunicationFieldNumber, nullptr},
      {LiveComm::kCarStateFieldNumber, nullptr},
      {LiveComm::kAnalogChFieldNumber, nullptr},
      {LiveComm::kCarStateFieldNumber, nullptr}};
};

#endif // TRX_QUEUEs_H