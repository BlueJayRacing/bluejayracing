#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Nanopb core includes.
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Include the generated headers from the installed library (from the old package).
#include "wsg_cal_com.pb.h"
#include "wsg_cal_data.pb.h"

int main(void)
{
    // --- Demonstrate cal_command_t encoding/decoding ---
    cal_command_t cmd = cal_command_t_init_zero;
    cmd.command = 1234;

    uint8_t buffer[128] = {0};
    pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&ostream, cal_command_t_fields, &cmd)) {
        printf("Error encoding cal_command_t: %s\n", PB_GET_ERROR(&ostream));
        return 1;
    }

    printf("Encoded cal_command_t to %zu bytes\n", ostream.bytes_written);

    cal_command_t decoded_cmd = cal_command_t_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(buffer, ostream.bytes_written);
    if (!pb_decode(&istream, cal_command_t_fields, &decoded_cmd)) {
        printf("Error decoding cal_command_t: %s\n", PB_GET_ERROR(&istream));
        return 1;
    }
    printf("Decoded cal_command_t.command = %u\n", decoded_cmd.command);

    // --- Demonstrate cal_data_t encoding/decoding ---
    // With fixed_count = true, the generated struct contains fixed arrays (40 elements each),
    // so no dynamic memory or count fields are needed.
    cal_data_t data = cal_data_t_init_zero;
    // Set only the first couple of values; the rest remain zero.
    data.adc_value[0] = 999;
    data.adc_value[1] = 1000;
    // Similarly, you can set data.dac_bias and data.voltage if needed.

    // Increase the output buffer size because fixed_count arrays encode all 40 elements.
    uint8_t buffer2[512] = {0};
    ostream = pb_ostream_from_buffer(buffer2, sizeof(buffer2));

    if (!pb_encode(&ostream, cal_data_t_fields, &data)) {
        printf("Error encoding cal_data_t: %s\n", PB_GET_ERROR(&ostream));
        return 1;
    }

    printf("Encoded cal_data_t to %zu bytes\n", ostream.bytes_written);

    cal_data_t decoded_data = cal_data_t_init_zero;
    istream = pb_istream_from_buffer(buffer2, ostream.bytes_written);
    if (!pb_decode(&istream, cal_data_t_fields, &decoded_data)) {
        printf("Error decoding cal_data_t: %s\n", PB_GET_ERROR(&istream));
        return 1;
    }

    // Print the first adc_value element.
    printf("Decoded cal_data_t: first adc_value = %u\n",
           decoded_data.adc_value[0]);

    return 0;
}
