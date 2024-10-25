#ifndef LIVE_COMM_FACTORY_H
#define LIVE_COMM_FACTORY_H

#include "baja_live_comm.pb.h"

// Make a LiveCommFactory which will synthesize a LiveComm object as a series of Observation objects is provided.
// If an Observation object with GPS is provided, and the previous Oberservation already has GPS added, then
// we'll append another observation to the internal LiveComm
class LiveCommFactory {
  public:
    // Add a single field from an observation to the LiveComm
    void add_observation(const Observation& observation);

    // Get the LiveComm object
    LiveComm get_live_comm();
    std::string get_serialized_live_comm();

  private:
    LiveComm live_comm;
};

#endif