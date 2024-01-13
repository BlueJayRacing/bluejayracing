#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <atomic>
#include <deque>
#include <thread>
#include <mutex>


// If cannot get to work, consider using oneTBB concurrent_bounded_queue<T, alloc> instead
// A thread-safe queue that can be accessed concurrently by enqueing
// and dequeing threads.

template <typename T>
class SafeQueue {
private:
  const int max_size;
  std::atomic<int> size;
  std::mutex dequeue_lock; // must be held while dequeueing
  std::mutex enqueue_lock; // must be held while enqueueing
  std::deque<T> data_queue;

public:

  SafeQueue(int max_size) : max_size (max_size) {}
  ~SafeQueue() {}

  int get_size() const {
    return size.load();
  }

  void enqueue(T data) {
    std::scoped_lock guard(enqueue_lock);
    if (size.load() < max_size) {
      this->data_queue.push_back(data);
      size += 1;
    };
  }

  T peek() {
    std::scoped_lock guard(dequeue_lock);
    if (size.load() <= 0) {
      std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
      return T();
    } 
    return this->data_queue.front();
  }


  T dequeue() {
    std::scoped_lock guard(dequeue_lock);
    if (size.load() <= 0) {
      std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
      return T();
    } 
    
    T d = this->data_queue.front();
    this->data_queue.pop_front();
    size -= 1;
    return d;
  }
};

#endif // MESSAGE_QUEUE_H