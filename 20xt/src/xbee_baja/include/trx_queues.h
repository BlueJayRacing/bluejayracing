#ifndef TRX_QUEUEs_H
#define TRX_QUEUEs_H

#include "safe_queue.h"
#include "baja_live_comm.pb.h"
#include <map>

// Other processes need to enqueue and deque data in its native
// format. The xbee-driver needs to dequeue data as a LiveComm
// struct. This class will handle the conversion between the two.
// This should preclude the need for switches, casts, or LiveComms
// anywhere outside the xbee-driver.

class TRXProtoQueues {
public:
  TRXProtoQueues();
  ~TRXProtoQueues();

  // Manage queue via wrapped data
  std::vector<int> field_ids_with_data();
  LiveComm dequeue(int proto_field_id);
  void enqueue(LiveComm data);
  int queue_size(int proto_field_id);

  // Enqueue data in its native format
  void enqueue(GPS data);
  void enqueue(Localization data);
  void enqueue(Communication data);
  void enqueue(Timestamp data);
  void enqueue(AnalogChannel data);
  void enqueue(CarState data);

  // Deque data in its native format
  GPS dequeue_gps();
  Localization dequeue_localization();
  Communication dequeue_communication();
  Timestamp dequeue_timestamp();
  AnalogChannel dequeue_analog_channel();
  CarState dequeue_car_state();

private:
  std::map<int, SafeQueue<LiveComm>*> field_id_to_queue = {
    {LiveComm::kGpsFieldNumber, nullptr},
    {LiveComm::kLocalizationFieldNumber, nullptr},
    {LiveComm::kCommunicationFieldNumber, nullptr},
    {LiveComm::kTimestampFieldNumber, nullptr},
    {LiveComm::kAnalogChFieldNumber, nullptr},
    {LiveComm::kCarStateFieldNumber, nullptr}
  }; // Hard coded switch
};

#endif // TRX_QUEUEs_H