#include <cassert>
#include <ctime>
#include <pthread.h>
#include <safe_queue.h>
#include "guard.h"

template <typename T>
SafeQueue<T>::SafeQueue(int max_size):size(0) {
  pthread_mutex_init(&dequeue_lock, NULL);
  pthread_mutex_init(&enqueue_lock, NULL);
}

// Destroy sempahore, lock, and messages
template <typename T>
SafeQueue<T>::~SafeQueue() {
  pthread_mutex_destroy(&dequeue_lock);
  pthread_mutex_destroy(&enqueue_lock);

  // Clean up dynamically allocated elements in the queue
  for (std ::deque<T*>::iterator it = data.begin();
      it != data.end(); it++) {
    delete (*it);
  }
}

template <typename T>
int SafeQueue<T>::size() const {
  return size.load();
}

// Enqueue a message pointer. Critical section.
template <typename T>
void SafeQueue<T>::enqueue(T *data) {
  Guard guard(enqueue_lock);
  data.push_back(data);
}

template <typename T>
Message *SafeQueue::dequeue() {
  struct timespec ts;

  // get the current time using clock_gettime:
  // we don't check the return value because the only reason
  // this call would fail is if we specify a clock that doesn't
  // exist. Compute a time one second in the future
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 1;

  if (sem_timedwait(&m_avail, &ts) == -1) {
    return nullptr;
  }
  
  Guard guard(d_lock);
  Message* msg = data.front();
  data.pop_front();
  return msg;
}