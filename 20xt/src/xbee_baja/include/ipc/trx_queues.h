#ifndef TRX_QUEUEs_H
#define TRX_QUEUEs_H

#include <map>

#include "ipc/safe_queue.h"
#include "baja_live_comm.pb.h"

static const int MAX_SIZE = 300;
// Multi queue containing all of the RX and TX data in its
// native format. Intended to be constructed by the supervisor
// thread. The supervisor thread will then pass the queues to
// the xbee-driver thread and other threads.

class TRXProtoQueues {
public:
  TRXProtoQueues();
  ~TRXProtoQueues();

  // Query number of elements queued
  int size_gps();
  int size_localization();
  int size_communication();
  int size_timestamp();
  int size_analog_channel();
  int size_car_state();

  // Enqueue data in its native format
  void enqueue(GPS data);
  void enqueue(Localization data);
  void enqueue(Communication data);
  void enqueue(Timestamp data);
  void enqueue(AnalogChannel data);
  void enqueue(CarState data);

  // Peek the first value
  GPS peek_gps();
  Localization peek_localization();
  Communication peek_communication();
  Timestamp peek_timestamp();
  AnalogChannel peek_analog_channel();
  CarState peek_car_state();

  // Deque data in its native format
  GPS dequeue_gps();
  Localization dequeue_localization();
  Communication dequeue_communication();
  Timestamp dequeue_timestamp();
  AnalogChannel dequeue_analog_channel();
  CarState dequeue_car_state();

private:
  std::map<std::string, void*> queues = {
    {"gps", nullptr},
    {"localization", nullptr},
    {"communication", nullptr},
    {"timestamp", nullptr},
    {"analog_channel", nullptr},
    {"car_state", nullptr}
  };
};

#endif // TRX_QUEUEs_H