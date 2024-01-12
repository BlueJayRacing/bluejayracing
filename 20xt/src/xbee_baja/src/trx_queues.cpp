#include "ipc/trx_queues.h"
#include "ipc/safe_queue.h"
#include "baja_live_comm.pb.h"

TRXProtoQueues::TRXProtoQueues(int max_queue_size) {
  queues[LiveComm::kGpsFieldNumber] = new SafeQueue<GPS>(max_queue_size);
  queues[LiveComm::kLocalizationFieldNumber] = new SafeQueue<Localization>(max_queue_size);
  queues[LiveComm::kCommunicationFieldNumber] = new SafeQueue<Communication>(max_queue_size);
  queues[LiveComm::kTimestampFieldNumber] = new SafeQueue<Timestamp>(max_queue_size);
  queues[LiveComm::kAnalogChFieldNumber] = new SafeQueue<AnalogChannel>(max_queue_size);
  queues[LiveComm::kCarStateFieldNumber] = new SafeQueue<CarState>(max_queue_size);
}

TRXProtoQueues::~TRXProtoQueues() {
  delete static_cast<SafeQueue<GPS>*>(queues[LiveComm::kGpsFieldNumber]);
  delete static_cast<SafeQueue<Localization>*>(queues[LiveComm::kLocalizationFieldNumber]);
  delete static_cast<SafeQueue<Communication>*>(queues[LiveComm::kCommunicationFieldNumber]);
  delete static_cast<SafeQueue<Timestamp>*>(queues[LiveComm::kTimestampFieldNumber]);
  delete static_cast<SafeQueue<AnalogChannel>*>(queues[LiveComm::kAnalogChFieldNumber]);
  delete static_cast<SafeQueue<CarState>*>(queues[LiveComm::kCarStateFieldNumber]);
}

int TRXProtoQueues::get_total_size() {
  int total_size = 0;
  total_size += static_cast<const SafeQueue<GPS>*>(queues[LiveComm::kGpsFieldNumber])->get_size();
  total_size += static_cast<const SafeQueue<Localization>*>(queues[LiveComm::kLocalizationFieldNumber])->get_size();
  total_size += static_cast<const SafeQueue<Communication>*>(queues[LiveComm::kCommunicationFieldNumber])->get_size();
  total_size += static_cast<const SafeQueue<Timestamp>*>(queues[LiveComm::kTimestampFieldNumber])->get_size();
  total_size += static_cast<const SafeQueue<AnalogChannel>*>(queues[LiveComm::kAnalogChFieldNumber])->get_size();
  total_size += static_cast<const SafeQueue<CarState>*>(queues[LiveComm::kCarStateFieldNumber])->get_size();
  return total_size;
}

template <typename T>
int TRXProtoQueues::get_size(int field_number) const {
  if (queues.at(field_number) == nullptr) {
    throw std::runtime_error("Queue not initialized");
  }
  return static_cast<SafeQueue<T>*>(queues.at(field_number))->get_size();
}

// Enqueue data in its native format
template <typename T>
void TRXProtoQueues::enqueue(int field_number, T data) {
  if (queues.at(field_number) == nullptr) {
    throw std::runtime_error("Queue not initialized");
  }
  static_cast<SafeQueue<T>*>(queues.at(field_number))->enqueue(data);
}

// Peek the first value
template <typename T>
T TRXProtoQueues::peek(int field_number) {
  if (queues.at(field_number) == nullptr) {
    throw std::runtime_error("Queue not initialized");
  }
  return static_cast<SafeQueue<T>*>(queues.at(field_number))->peek();
}

// Deque data in its native format
template <typename T>
T TRXProtoQueues::dequeue(int field_number) {
  if (queues.at(field_number) == nullptr) {
    throw std::runtime_error("Queue not initialized");
  }
  return static_cast<SafeQueue<T>*>(queues.at(field_number))->dequeue();
}

