#include "baja_live_comm.pb.h"
#include "helpers/live_comm_factory.h"
#include "proto_helpers.h"


LiveCommFactory::LiveCommFactory()
{
  this->live_comm = LiveComm();
  this->current_observation = this->live_comm.add_observations();
}

void LiveCommFactory::add_observation(const Observation& new_data)
{
  
  std::vector<int> set_field_ids = BajaProtoHelpers::get_set_field_ids(new_data);
  for (int field_id : set_field_ids)
  {
    _add_observation(field_id, new_data);
  }
}

void LiveCommFactory::_add_observation(int field_id, const Observation& new_data)
{
  // Get a reflection of the current observation to check if it has the field
  const google::protobuf::FieldDescriptor *field = Observation::GetDescriptor()->FindFieldByNumber(field_id);
  if (field == NULL)
  {
    return; // Field not valid
  }

  // If the current observation already has the field, we need to start a new observation
  const google::protobuf::Reflection *curr_obs_reflection = this->current_observation->GetReflection();
  if (curr_obs_reflection->HasField(*this->current_observation, field))
  {
    this->current_observation = this->live_comm.add_observations();
  }

  // Set the current observation field to a copy of the value from new_data_reflection
  const google::protobuf::Message &new_data_msg = new_data.GetReflection()->GetMessage(new_data, field);
  curr_obs_reflection->MutableMessage(this->current_observation, field)->CopyFrom(new_data_msg);
}

LiveComm LiveCommFactory::get_live_comm()
{
  return live_comm;
}
