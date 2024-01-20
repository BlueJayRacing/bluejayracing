#ifndef PROTO_HELPERS_BAJA_H
#define PROTO_HELPERS_BAJA_H

#include "baja_live_comm.pb.h"

namespace BajaProtoHelpers {
  const std::vector<int> OBSERVATION_FIELD_IDS = {
        Observation::kGpsFieldNumber,
        Observation::kLocalizationFieldNumber,
        Observation::kCommunicationFieldNumber,
        Observation::kAnalogChFieldNumber,
        Observation::kCarStateFieldNumber,
    };

  std::vector<int> get_set_field_ids(Observation& obs) {
    // Does not check the timestamp field
    std::vector<int> set_field_ids;
    const google::protobuf::Reflection *reflection = obs.GetReflection();

    for (int field_id : OBSERVATION_FIELD_IDS) {
      const google::protobuf::FieldDescriptor *field = obs.GetDescriptor()->FindFieldByNumber(field_id);
      if (field != nullptr)
      {
        if (reflection->HasField(obs, field))
        {
          set_field_ids.push_back(field_id);
        }
      }
    }
    return set_field_ids;
  }
}


#endif