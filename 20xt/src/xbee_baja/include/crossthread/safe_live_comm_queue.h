#ifndef SAFE_LIVE_COMM_QUEUE_H
#define SAFE_LIVE_COMM_QUEUE_H

#include <atomic>
#include <deque>
#include <thread>
#include <mutex>

#include "interfaces/live_comm_queue.h"

// This is a thread-safe queue that can be used to pass data between threads.
// Be wary when using front() and then dequeue() as another consumer might have
// stolen your data
class SafeLiveCommQueue : public LiveCommQueue {
private:
  const int max_size;
  std::atomic<int> tail; // Oldest element
  std::atomic<int> head; // Newest element
  std::atomic<int> qlen;

  std::mutex dequeue_lock; // must be held while dequeueing
  std::mutex enqueue_lock; // must be held while enqueueing
  Observation* data_queue;

public:

  SafeLiveCommQueue(int max_size);

  ~SafeLiveCommQueue();

  int size();

  // Critical Section
  bool enqueue(Observation data);

  // Critical Section
  Observation front();

  // Critical Section
  Observation dequeue();
};

#endif // MESSAGE_QUEUE_H