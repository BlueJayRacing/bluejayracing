#include <stdio.h>
#include <esp_system.h>
#include <esp_log.h>
#include <time.h>
#include <freertos/FreeRTOS.h>

#include "test_proto_1.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    uint8_t buffer[128];
    int message_length;
    int num = 0;

    while (true) {
        {
            simpleMessage enc_message;
            pb_ostream_t o_stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

            enc_message.lucky_number = num;
            enc_message.lucky_number_2 = 2;

            pb_encode(&o_stream, simpleMessage_fields, &enc_message);
            message_length = o_stream.bytes_written;
        }

        {
            simpleMessage dec_message;
            pb_istream_t i_stream = pb_istream_from_buffer(buffer, message_length);
            pb_decode(&i_stream, simpleMessage_fields, &dec_message);
            ESP_LOGI(TAG, "Decoded value: %d", (int) dec_message.lucky_number); 
            num++;   
        }

        vTaskDelay(100);
    }
}
