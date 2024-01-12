#ifndef TRX_QUEUEs_H
#define TRX_QUEUEs_H

#include <map>
#include <iostream>

#include "ipc/safe_queue.h"
#include "ipc/safe_queue_base.h"
#include "baja_live_comm.pb.h"

static const int MAX_SIZE = 300;
// Multi queue containing all of the RX and TX data in its
// native format. Intended to be constructed by the supervisor
// thread. The supervisor thread will then pass the queues to
// the xbee-driver thread and other threads.

class TRXProtoQueues
{
public:
  enum FieldID
  {
    GPS_QUEUE_ID = 1,
    LOCALIZATION_QUEUE_ID = 2,
    COMMUNICATION_QUEUE_ID = 3,
    TIMESTAMP_QUEUE_ID = 4,
    ANALOG_CH_QUEUE_ID = 5,
    CAR_STATE_QUEUE_ID = 6
  };

private:
  std::map<FieldID, SafeQueueBase *> queues = {
      {GPS_QUEUE_ID, nullptr},
      {LOCALIZATION_QUEUE_ID, nullptr},
      {COMMUNICATION_QUEUE_ID, nullptr},
      {TIMESTAMP_QUEUE_ID, nullptr},
      {ANALOG_CH_QUEUE_ID, nullptr},
      {CAR_STATE_QUEUE_ID, nullptr}};

   template <typename T>
   SafeQueue<T>* get_queue(FieldID queue_id)
   {
     if (queues.at(queue_id) == nullptr)
     {
       throw std::runtime_error("Queue not initialized");
     }
     auto queue = static_cast<SafeQueue<T> *>(queues.at(queue_id));
     if (queue == nullptr)
     {
       throw std::runtime_error("Cannot cast queue. Hint: check queue ID matches expected type");
     }
     return queue;
   }

public:
  TRXProtoQueues(int max_queue_size)
  {
    queues[GPS_QUEUE_ID] = dynamic_cast<SafeQueueBase *>(new SafeQueue<GPS>(max_queue_size));
    queues[LOCALIZATION_QUEUE_ID] = dynamic_cast<SafeQueueBase *>(new SafeQueue<Localization>(max_queue_size));
    queues[COMMUNICATION_QUEUE_ID] = dynamic_cast<SafeQueueBase *>(new SafeQueue<Communication>(max_queue_size));
    queues[TIMESTAMP_QUEUE_ID] = dynamic_cast<SafeQueueBase *>(new SafeQueue<Timestamp>(max_queue_size));
    queues[ANALOG_CH_QUEUE_ID] = dynamic_cast<SafeQueueBase *>(new SafeQueue<AnalogChannel>(max_queue_size));
    queues[CAR_STATE_QUEUE_ID] = dynamic_cast<SafeQueueBase *>(new SafeQueue<CarState>(max_queue_size));
  }
  ~TRXProtoQueues()
  {
    delete dynamic_cast<SafeQueue<GPS> *>(queues[GPS_QUEUE_ID]);
    delete dynamic_cast<SafeQueue<Localization> *>(queues[LOCALIZATION_QUEUE_ID]);
    delete dynamic_cast<SafeQueue<Communication> *>(queues[COMMUNICATION_QUEUE_ID]);
    delete dynamic_cast<SafeQueue<Timestamp> *>(queues[TIMESTAMP_QUEUE_ID]);
    delete dynamic_cast<SafeQueue<AnalogChannel> *>(queues[ANALOG_CH_QUEUE_ID]);
    delete dynamic_cast<SafeQueue<CarState> *>(queues[CAR_STATE_QUEUE_ID]);
  }

  int get_total_size()
  {
    int total_size = 0;
    total_size += dynamic_cast<SafeQueue<GPS> *>(queues[GPS_QUEUE_ID])->get_size();
    total_size += dynamic_cast<SafeQueue<Localization> *>(queues[LOCALIZATION_QUEUE_ID])->get_size();
    total_size += dynamic_cast<SafeQueue<Communication> *>(queues[COMMUNICATION_QUEUE_ID])->get_size();
    total_size += dynamic_cast<SafeQueue<Timestamp> *>(queues[TIMESTAMP_QUEUE_ID])->get_size();
    total_size += dynamic_cast<SafeQueue<AnalogChannel> *>(queues[ANALOG_CH_QUEUE_ID])->get_size();
    total_size += dynamic_cast<SafeQueue<CarState> *>(queues[CAR_STATE_QUEUE_ID])->get_size();
    return total_size;
  }

  // Query number of elements queued
  template <typename T>
  int get_size(FieldID queue_id)
  {
    SafeQueue<T>* queue = get_queue<T>(queue_id);
    return queue->get_size();
  }

  // Enqueue data in its native format
  template <typename T>
  void enqueue(FieldID queue_id, T data)
  {
    SafeQueue<T>* queue = get_queue<T>(queue_id);
    queue->enqueue(data);
  }

  // Peek the first value
  template <typename T>
  T peek(FieldID queue_id)
  {
    SafeQueue<T>* queue = get_queue<T>(queue_id);
    return queue->peek();
  }

  // Deque data in its native format
  template <typename T>
  T dequeue(FieldID queue_id)
  {
    SafeQueue<T>* queue = get_queue<T>(queue_id);
    return queue->dequeue();
  }
};

#endif // TRX_QUEUEs_H