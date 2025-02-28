#include <Arduino.h>

#include "ad717x.hpp"

#define ADC_INT_PIN 28
#define ADC_CS_PIN 10

#define CENTER 0x800000

#define NUMBER_OF_CHANNELS 11

AD717X ad7175;
int32_t data_val;

double voltage_val;

void setup()
{
    delay(2000);
    Serial.begin(115200);
    SPI.begin();

    pinMode(1, OUTPUT);

    ad717x_init_param_t ad7175_params;

    ad717x_analog_inputs_t chan_0_inputs;
    chan_0_inputs.ainp.pos_input = AIN10;
    chan_0_inputs.ainp.neg_input = AIN9;

    ad717x_channel_config_t chan_config = {true, false, false, AVDD_AVSS};
    ad717x_filter_config_t filt_con       = {false, false, SPS27_DB47_MS36P7, SINC3, SPS_250000};
    ad717x_setup_t setup                  = {1, chan_config, filt_con};
    ad717x_channel_map_t chan_0_map       = {true, 0, chan_0_inputs};

    ad7175_params.active_device = ID_AD7175_8;
    ad7175_params.ref_en        = true;
    ad7175_params.mode          = CONTINUOUS;
    ad7175_params.stat_on_read_en = true;

    for (int i = 0; i < NUMBER_OF_CHANNELS; i++) {
        ad7175_params.chan_map.push_back(chan_0_map);
    }

    ad7175_params.setups.push_back(setup);

    pinMode(ADC_INT_PIN, INPUT);

    ad7175.init(ad7175_params, &SPI, ADC_CS_PIN);
}

uint64_t channel_count[NUMBER_OF_CHANNELS];

void loop()
{
    for (int i = 0; i < NUMBER_OF_CHANNELS; i++) {
        channel_count[i] = 0;
    }

    uint32_t start_time = millis();
    while (millis() - start_time < 5000) {
        ad717x_data_t data;

        delayMicroseconds(20);
        ad7175.contConvReadData(&data);

        channel_count[data.status.active_channel]++;
    }
    
    uint64_t total = 0;

    for (int i = 0; i < NUMBER_OF_CHANNELS; i++) {
        Serial.print("Channel ");
        Serial.print(i);
        Serial.print(" Count: ");
        total += channel_count[i];
        Serial.println(channel_count[i]);
    }
    Serial.print("Total Count per second: ");
    Serial.println(total / 5);
}