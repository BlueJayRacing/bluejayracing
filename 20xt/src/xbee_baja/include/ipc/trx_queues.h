#ifndef TRX_QUEUEs_H
#define TRX_QUEUEs_H

#include <map>

#include "ipc/safe_queue.h"
#include "baja_live_comm.pb.h"

static const int MAX_SIZE = 300;
// Multi queue containing all of the RX and TX data in its
// native format. Intended to be constructed by the supervisor
// thread. The supervisor thread will then pass the queues to
// the xbee-driver thread and other threads.

class TRXProtoQueues {
public:
  TRXProtoQueues(int max_queue_size);
  ~TRXProtoQueues();

  // Query number of elements queued
  template <typename T>
  int get_size(int field_number) const;

  // Enqueue data in its native format
  template <typename T>
  void enqueue(int field_number, T data);

  // Peek the first value
  template <typename T>
  T peek(int field_number);

  // Deque data in its native format
  template <typename T>
  T dequeue(int field_number);

  // Get the total size
  int get_total_size();

private:
  std::map<int, void*> queues = {
    {LiveComm::kGpsFieldNumber, nullptr},
    {LiveComm::kLocalizationFieldNumber, nullptr},
    {LiveComm::kCommunicationFieldNumber, nullptr},
    {LiveComm::kTimestampFieldNumber, nullptr},
    {LiveComm::kAnalogChFieldNumber, nullptr},
    {LiveComm::kCarStateFieldNumber, nullptr}
  };
};

#endif // TRX_QUEUEs_H