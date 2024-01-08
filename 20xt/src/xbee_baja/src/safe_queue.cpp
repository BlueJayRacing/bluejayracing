#include <cassert>
#include <ctime>
#include <pthread.h>
#include <safe_queue.h>
#include "guard.h"

template <typename T>
SafeQueue<T>::SafeQueue(int max_size) {
  pthread_mutex_init(&dequeue_lock, NULL);
  pthread_mutex_init(&enqueue_lock, NULL);
  this->max_size = max_size;
}

// Destroy sempahore, lock, and messages
template <typename T>
SafeQueue<T>::~SafeQueue() {
  pthread_mutex_destroy(&dequeue_lock);
  pthread_mutex_destroy(&enqueue_lock);

  // Clean up dynamically allocated elements in the queue
  for (const auto& item : data) {
    delete item;
  }
}

template <typename T>
int SafeQueue<T>::get_size() const {
  return size.get();
}

// Enqueue a message pointer. Critical section.
template <typename T>
void SafeQueue<T>::enqueue(T data) {
  Guard guard(enqueue_lock);

  T* data_ptr = new T(data);
  if (size.get() < max_size) {
    data.push_back(data_ptr);
    size += 1;
  }
}

// Dequeue a message pointer. Critical section.
template <typename T>
T SafeQueue<T>::dequeue() {
  Guard guard(dequeue_lock);
  if (data.empty()) {
    return NULL;
  }
  
  T d = *data.front();
  data.pop_front();
  size -= 1;
  return d;
}



