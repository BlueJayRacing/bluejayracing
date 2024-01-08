#ifndef GUARD_H
#define GUARD_H

#include <pthread.h>

class Guard {
public:
  Guard(pthread_mutex_t &lock)
    : lock(lock) {
    pthread_mutex_lock(&lock);
  }

  ~Guard() {
    pthread_mutex_unlock(&lock);
  }

private:
  Guard(const Guard &);
  Guard &operator=(const Guard &);
  pthread_mutex_t &lock;
};

#endif // GUARD_H