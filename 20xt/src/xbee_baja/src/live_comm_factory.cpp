#include "baja_live_comm.pb.h"
#include "ipc/live_comm_factory.h"


LiveCommFactory::LiveCommFactory()
{
  live_comm = LiveComm();
  current_observation = live_comm.add_observations();
}

void LiveCommFactory::add_observation(int field_id, Observation new_data)
{
  // Get a reflection of the current observation to check if it has the field
  const google::protobuf::FieldDescriptor *field = Observation::GetDescriptor()->FindFieldByNumber(field_id);
  if (field == NULL)
  {
    return; // Field not valid
  }

  // If the current observation already has the field, we need to start a new observation
  const google::protobuf::Reflection *curr_obs_reflection = current_observation->GetReflection();
  if (curr_obs_reflection->HasField(*current_observation, field))
  {
    current_observation = live_comm.add_observations();
  }

  // Set the current observation field to a copy of the value from new_data_reflection
  const google::protobuf::Message &new_data_msg = new_data.GetReflection()->GetMessage(new_data, field);
  curr_obs_reflection->MutableMessage(current_observation, field)->CopyFrom(new_data_msg);
}

LiveComm LiveCommFactory::get_live_comm()
{
  return live_comm;
}
