#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

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
  LiveComm* data_queue;

public:

  SafeLiveCommQueue(int max_size) : max_size (max_size), qlen(0), head(-1) {
    data_queue = new LiveComm[max_size];
  }

  ~SafeLiveCommQueue() {
    delete[] data_queue;
  }

  int size() {
    return qlen.load();
  }

  // Critical Section
  bool enqueue(LiveComm data) {
    std::scoped_lock guard(enqueue_lock);
    if (qlen.load() >= max_size) {
      return false;
    };

    qlen += 1; // Claim more length below filling it
    head = (head.load() + 1) % max_size;
    data_queue[head.load()] = data;
    return true;
  }

  // Critical Section
  LiveComm front() {
    std::scoped_lock guard(dequeue_lock);
    if (qlen.load() <= 0) {
      std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
      return LiveComm();
    } 

    return data_queue[tail.load()];
  }


  // Critical Section
  LiveComm dequeue() {
    std::scoped_lock guard(dequeue_lock);
    if (qlen.load() <= 0) {
      std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
      return LiveComm();
    } 
    
    LiveComm data = data_queue[tail.load()];
    tail = (tail.load() + 1) % max_size;
    qlen -= 1; // Must reduce length after retrieving data
    return data;
  }
};

#endif // MESSAGE_QUEUE_H