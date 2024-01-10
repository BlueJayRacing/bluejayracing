#include <thread>
#include <mutex>
#include "ipc/safe_queue.h"

template <typename T>
SafeQueue<T>::SafeQueue(int max_size) : max_size (max_size) {
}

// Destroy sempahore, lock, and messages
template <typename T>
SafeQueue<T>::~SafeQueue() {
  // Clean up dynamically allocated elements in the queue
  for (const auto& item : data) {
    delete item;
  }
}

template <typename T>
int SafeQueue<T>::get_size() const {
  return size.load();
}

// Enqueue a message pointer. Critical section.
template <typename T>
void SafeQueue<T>::enqueue(T data) {
  std::scoped_lock guard(enqueue_lock);
  
  T* data_ptr = new T(data);
  if (size.load() < max_size) {
    data.push_back(data_ptr);
    size += 1;
  }
}

// Dequeue a message pointer. Critical section.
template <typename T>
T SafeQueue<T>::dequeue() {
  std::scoped_lock guard(dequeue_lock);

  if (data.empty()) {
    return NULL;
  } 
  
  T d = *data.front();
  data.pop_front();
  size -= 1;
  return d;
}



