#include "trx_queues.h"
#include "baja_live_comm.pb.h"

TRXProtoQueues::TRXProtoQueues() {

}

TRXProtoQueues::~TRXProtoQueues() {

}

// Query number of elements queued
int TRXProtoQueues::size_gps() {
  return 0;
}

int TRXProtoQueues::size_localization() {
  return 0;
}

int TRXProtoQueues::size_communication() {
  return 0;
}

int TRXProtoQueues::size_timestamp() {
  return 0;
}

int TRXProtoQueues::size_analog_channel() {
  return 0;
}

int TRXProtoQueues::size_car_state() {
  return 0;
}



// Enqueue data in its native format
void TRXProtoQueues::enqueue(GPS data) {

}

void TRXProtoQueues::enqueue(Localization data) {

}

void TRXProtoQueues::enqueue(Communication data) {

}

void TRXProtoQueues::enqueue(Timestamp data) {

}

void TRXProtoQueues::enqueue(AnalogChannel data) {

}

void TRXProtoQueues::enqueue(CarState data) {

}


// Peek the first value
GPS TRXProtoQueues::peek_gps() {
  return GPS();
}

Localization TRXProtoQueues::peek_localization() {
  return Localization();
}

Communication TRXProtoQueues::peek_communication() {
  return Communication();
}

Timestamp TRXProtoQueues::peek_timestamp() {
  return Timestamp();
}

AnalogChannel TRXProtoQueues::peek_analog_channel() {
  return AnalogChannel();
}

CarState TRXProtoQueues::peek_car_state() {
  return CarState();
}


// Deque data in its native format
GPS TRXProtoQueues::dequeue_gps() {
  return GPS();
}

Localization TRXProtoQueues::dequeue_localization() {
  return Localization();
}

Communication TRXProtoQueues::dequeue_communication() {
  return Communication();
}

Timestamp TRXProtoQueues::dequeue_timestamp() {
  return Timestamp();
}

AnalogChannel TRXProtoQueues::dequeue_analog_channel() {
  return AnalogChannel();
}

CarState TRXProtoQueues::dequeue_car_state() {
  return CarState();
}


