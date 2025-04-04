#include <esp_system.h>

#include <esp_data_chunk.pb.h>
#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <array>

namespace hardEncoder
{

int encodeESPDataChunk(ESPDataChunk& data_chunk, std::array<uint8_t, 12000>& buffer);

}; // namespace hardEncoder