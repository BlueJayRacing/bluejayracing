#include <esp_system.h>

#include <wsg_drive_data.pb.h>
#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <array>

namespace hardEncoder
{

int encodeDriveData(wsg_drive_data_t& data_chunk, std::array<uint8_t, 12000>& buffer);

}; // namespace hardEncoder