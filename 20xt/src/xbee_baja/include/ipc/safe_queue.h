#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <atomic>
#include <deque>
#include <thread>
#include <mutex>

#include "interfaces/live_comm_queue.h"


// If cannot get to work, consider using oneTBB concurrent_bounded_queue<T, alloc> instead
// A thread-safe queue that can be accessed concurrently by enqueing
// and dequeing threads.

class SafeQueue : public LiveCommQueue {
private:
  const int max_size;
  std::atomic<int> curr_size;
  std::mutex dequeue_lock; // must be held while dequeueing
  std::mutex enqueue_lock; // must be held while enqueueing
  std::deque<LiveComm> data_queue;

public:

  SafeQueue(int max_size) : max_size (max_size) {}
  ~SafeQueue() {}

  int size() {
    return curr_size.load();
  }

  bool enqueue(LiveComm data) {
    std::scoped_lock guard(enqueue_lock);
    if (curr_size.load() >= max_size) {
      return false;
    };

    curr_size += 1;
    this->data_queue.push_back(data);
    return true;
  }

  LiveComm front() {
    std::scoped_lock guard(dequeue_lock);
    if (curr_size.load() <= 0) {
      std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
      return LiveComm();
    } 
    return this->data_queue.front();
  }


  LiveComm dequeue() {
    std::scoped_lock guard(dequeue_lock);
    if (curr_size.load() <= 0) {
      std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
      return LiveComm();
    } 
    
    LiveComm d = this->data_queue.front();
    this->data_queue.pop_front();
    curr_size -= 1;
    return d;
  }
};

#endif // MESSAGE_QUEUE_H