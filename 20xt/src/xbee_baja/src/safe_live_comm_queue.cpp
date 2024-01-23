#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <atomic>
#include <deque>
#include <thread>
#include <mutex>

#include "interfaces/live_comm_queue.h"
#include "crossthread/safe_live_comm_queue.h"
#include "baja_live_comm.pb.h"

// TODO: rename SafeObservationQueue to SafeObservationQueue
SafeObservationQueue::SafeObservationQueue(int max_size) : max_size(max_size), qlen(0), head(-1)
{
  this->data_queue = new Observation[max_size];
}

SafeObservationQueue::~SafeObservationQueue()
{
  delete[] this->data_queue;
}

int SafeObservationQueue::size()
{
  return qlen.load();
}

// Critical Section
bool SafeObservationQueue::enqueue(Observation data)
{
  std::scoped_lock guard(enqueue_lock);
  if (qlen.load() >= max_size)
  {
    return false;
  };

  qlen += 1; // Claim more length below filling it
  head = (head.load() + 1) % max_size;
  data_queue[head.load()] = data;
  return true;
}

// Critical Section
Observation SafeObservationQueue::front()
{
  std::scoped_lock guard(dequeue_lock);
  if (qlen.load() <= 0)
  {
    std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
    return Observation();
  }

  return data_queue[tail.load()];
}

// Critical Section
Observation SafeObservationQueue::dequeue()
{
  std::scoped_lock guard(dequeue_lock);
  if (qlen.load() <= 0)
  {
    std::cerr << "ERROR: queue is empty, returning default value" << std::endl;
    return Observation();
  }

  Observation data = data_queue[tail.load()];
  tail = (tail.load() + 1) % max_size;
  qlen -= 1; // Must reduce length after retrieving data
  return data;
}

#endif // MESSAGE_QUEUE_H