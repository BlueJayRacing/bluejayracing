#ifndef LIVE_COMM_FACTORY_H
#define LIVE_COMM_FACTORY_H

#include "baja_live_comm.pb.h"

// Make a LiveCommFactory which will synthesize a LiveComm object as a series of Observation objects is provided. 
// If an Observation object with GPS is provided, and the previous Oberservation already has GPS added, then 
// we'll append another observation to the internal LiveComm
class LiveCommFactory {
public:
  LiveCommFactory();
  ~LiveCommFactory() {}

  // Add a single field from an observation to the LiveComm
  void add_observation(const Observation& observation);

  // Get the LiveComm object
  LiveComm get_live_comm();

private:
  LiveComm live_comm;
  Observation *current_observation;
  void _add_observation(int field_id, const Observation& new_data);
};

#endif