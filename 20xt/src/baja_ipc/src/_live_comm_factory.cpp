#include "baja_live_comm.pb.h"
#include "live_comm_factory.h"
#include "proto_helpers.h"

void LiveCommFactory::add_observation(const Observation& new_data)
{
  Observation* buffer = this->live_comm.add_observations();
  buffer->CopyFrom(new_data);
}

std::string LiveCommFactory::get_serialized_live_comm() {
  return this->live_comm.SerializeAsString();
}