#include <hardEncoder.hpp>

namespace hardEncoder
{

static void encodeESPSample(ESPSample& data_sample, pb_ostream_t& stream) {
    if (!pb_encode_tag(&stream, PB_WT_32BIT, ESPSample_value_tag)) {
        return;
    }
    if (!pb_encode_fixed32(&stream, &data_sample.value)) {
        return;
    }

    if (!pb_encode_tag(&stream, PB_WT_VARINT, ESPSample_timestamp_delta_tag)) {
        return;
    }
    if (!pb_encode_varint(&stream, data_sample.timestamp_delta)) {
        return;
    }
}

int encodeESPDataChunk(ESPDataChunk& data_chunk, std::array<uint8_t, 12000>& buffer)
{
    pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());

    // Encode MAC Address
    if (!pb_encode_tag(&stream, PB_WT_STRING, ESPDataChunk_mac_address_tag)) {
        return -1;
    }
    if (!pb_encode_string(&stream, (pb_byte_t*)data_chunk.mac_address, strlen(data_chunk.mac_address))) {
        return -1;
    }

    // Encode Base Timestamp
    if (!pb_encode_tag(&stream, PB_WT_64BIT, ESPDataChunk_base_timestamp_tag)) {
        return -1;
    }
    if (!pb_encode_fixed64(&stream, &data_chunk.base_timestamp)) {
        return -1;
    }

    for (int i = 0; i < 1000; i++) {
        if (!pb_encode_tag(&stream, PB_WT_STRING, ESPDataChunk_samples_tag)) {
            return -1;
        }

        uint8_t sub_buffer[20];
        pb_ostream_t sub_stream = pb_ostream_from_buffer(sub_buffer, sizeof(sub_buffer));

        encodeESPSample(data_chunk.samples[i], sub_stream);

        if (!pb_encode_varint(&stream, sub_stream.bytes_written)) {
            return -1;
        }

        // 4. Write submessage bytes
        if (!pb_write(&stream, sub_buffer, sub_stream.bytes_written)) {
            return -1;
        }
    }

    // Encode DAC Bias
    if (!pb_encode_tag(&stream, PB_WT_VARINT, ESPDataChunk_dac_bias_tag)) {
        return -1;
    }
    if (!pb_encode_varint(&stream, data_chunk.dac_bias)) {
        return -1;
    }

    // Encode Excitation Voltage
    if (!pb_encode_tag(&stream, PB_WT_VARINT, ESPDataChunk_excitation_voltage_tag)) {
        return -1;
    }
    if (!pb_encode_varint(&stream, data_chunk.excitation_voltage)) {
        return -1;
    }

    // Encode Sample Channel ID
    if (!pb_encode_tag(&stream, PB_WT_VARINT, ESPDataChunk_sample_channel_id_tag)) {
        return -1;
    }
    if (!pb_encode_varint(&stream, data_chunk.sample_channel_id)) {
        return -1;
    }

    return stream.bytes_written;
}

}; // namespace hardEncoder