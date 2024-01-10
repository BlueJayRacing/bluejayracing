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
public:
  SafeQueue(int max_size);
  ~SafeQueue();

  int get_size() const;
  void enqueue(T data);
  T dequeue();

private:
  const int max_size;
  std::atomic<int> size;

  std::mutex dequeue_lock; // must be held while dequeueing
  std::mutex enqueue_lock; // must be held while enqueueing
  // THE MESSAGES IN THE QUEUE MUST BE DYNAMICALLY ALLOCATED
  std::deque<T *> data;
};

#endif // MESSAGE_QUEUE_H