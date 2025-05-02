#include <hardEncoder.hpp>
#include <esp_log.h>

static const char* TAG = "hard_encoder";

namespace hardEncoder
{

int encodeDriveData(wsg_drive_data_t& data_chunk, std::array<uint8_t, 12000>& buffer)
{
    pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), 12000);

    // Encode MAC Address
    if (!pb_encode_tag(&stream, PB_WT_STRING, wsg_drive_data_t_mac_address_tag)) {
        return -1;
    }
    if (!pb_encode_string(&stream, (pb_byte_t*)data_chunk.mac_address, 17)) {
        return -1;
    }

    // ESP_LOGI(TAG, "After MAC: %u", stream.bytes_written);

    // Encode Base Timestamp
    if (!pb_encode_tag(&stream, PB_WT_64BIT, wsg_drive_data_t_base_timestamp_tag)) {
        return -1;
    }
    if (!pb_encode_fixed64(&stream, &data_chunk.base_timestamp)) {
        return -1;
    }

    // ESP_LOGI(TAG, "After Timestamp: %u", stream.bytes_written);

    if (!pb_encode_tag(&stream, PB_WT_STRING, wsg_drive_data_t_values_tag)) {
        return -1;
    }
    if(!pb_encode_varint(&stream, 4000)) {
        return -1;
    }

    for (int i = 0; i < 1000; i++) {
        if (!pb_encode_fixed32(&stream, &data_chunk.values[i])) {
            return -1;
        }
    }

    // ESP_LOGI(TAG, "After Values: %u", stream.bytes_written);

    if (!pb_encode_tag(&stream, PB_WT_STRING, wsg_drive_data_t_timestamp_deltas_tag)) {
        return -1;
    }
    if(!pb_encode_varint(&stream, 4000)) {
        return -1;
    }

    for (int i = 0; i < 1000; i++) {
        if (!pb_encode_fixed32(&stream, &data_chunk.timestamp_deltas[i])) {
            return -1;
        }
    }

    // ESP_LOGI(TAG, "After timestamp deltas: %u", stream.bytes_written);

    // Encode DAC Bias
    if (!pb_encode_tag(&stream, PB_WT_VARINT, wsg_drive_data_t_dac_bias_tag)) {
        return -1;
    }
    if (!pb_encode_varint(&stream, data_chunk.dac_bias)) {
        return -1;
    }

    // Encode Excitation Voltage
    if (!pb_encode_tag(&stream, PB_WT_VARINT, wsg_drive_data_t_excitation_voltage_tag)) {
        return -1;
    }
    if (!pb_encode_varint(&stream, data_chunk.excitation_voltage)) {
        return -1;
    }

    // Encode Sample Channel ID
    if (!pb_encode_tag(&stream, PB_WT_VARINT, wsg_drive_data_t_sample_channel_id_tag)) {
        return -1;
    }
    if (!pb_encode_varint(&stream, data_chunk.sample_channel_id)) {
        return -1;
    }

    return stream.bytes_written;
}

}; // namespace hardEncoder