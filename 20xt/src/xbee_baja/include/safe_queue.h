#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <atomic>
#include <queue>
#include <pthread.h>
#include <semaphore.h>

// A thread-safe queue that can be accessed concurrently by enqueing
// and dequeing threads.
template <typename T>
class SafeQueue {
public:
  SafeQueue(int max_size);
  ~SafeQueue();

  int size() const;
  void enqueue(T data);
  T dequeue();

private:
  pthread_mutex_t dequeue_lock; // must be held while dequeueing
  pthread_mutex_t enqueue_lock; // must be held while enqueueing
  std::atomic<int> size;
  // THE MESSAGES IN THE QUEUE MUST BE DYNAMICALLY ALLOCATED
  std::queue<T *> data;
};

#endif // MESSAGE_QUEUE_H