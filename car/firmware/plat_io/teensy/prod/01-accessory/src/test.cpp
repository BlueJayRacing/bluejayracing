#include <test.hpp>

#define ADC_CS_PIN 10

#define D1_PIN  27
#define D2_PIN  26
#define D3_PIN  39
#define D4_PIN  38
#define D5_PIN  40
#define D6_PIN  41

#define DB44_AIN1 AIN6
#define DB44_AIN2 AIN8
#define DB44_AIN3 AIN10
#define DB44_AIN4 AIN12
#define DB44_AIN5 AIN14
#define DB44_AIN6 AIN7

#define NUMBER_ANALOG_CHANNELS 6

void Test::readAnalogChannels(void)
{
    SPI.begin();

    ad717x_init_param_t ad7175_params;

    ad717x_analog_inputs_t chan_inputs[NUMBER_ANALOG_CHANNELS];
    chan_inputs[0].ainp.pos_input = DB44_AIN1;
    chan_inputs[0].ainp.neg_input = REF_M;

    chan_inputs[1].ainp.pos_input = DB44_AIN2;
    chan_inputs[1].ainp.neg_input = REF_M;

    chan_inputs[2].ainp.pos_input = DB44_AIN3;
    chan_inputs[2].ainp.neg_input = REF_M;

    chan_inputs[3].ainp.pos_input = DB44_AIN4;
    chan_inputs[3].ainp.neg_input = REF_M;

    chan_inputs[4].ainp.pos_input = DB44_AIN5;
    chan_inputs[4].ainp.neg_input = REF_M;

    chan_inputs[5].ainp.pos_input = DB44_AIN6;
    chan_inputs[5].ainp.neg_input = REF_M;

    ad717x_channel_config_t chan_config = {true, false, false, AVDD_AVSS};
    ad717x_filter_config_t filt_con     = {false, false, SPS27_DB47_MS36P7, SINC5_SINC1, SPS_1000};
    ad717x_setup_t setup                = {1, chan_config, filt_con};

    ad7175_params.active_device   = ID_AD7175_8;
    ad7175_params.ref_en          = true;
    ad7175_params.mode            = CONTINUOUS;
    ad7175_params.stat_on_read_en = true;

    for (int i = 0; i < NUMBER_ANALOG_CHANNELS; i++) {
        ad717x_channel_map_t chan_map = {true, 0, chan_inputs[i]};
        ad7175_params.chan_map.push_back(chan_map);
    }

    ad7175_params.setups.push_back(setup);

    ad7175.init(ad7175_params, &SPI, ADC_CS_PIN);

    int data_count = 0;
    uint32_t data_values[NUMBER_ANALOG_CHANNELS];

    for (;;) {
        ad717x_data_t data;

        ad7175.waitForReady(0xFFFFFFFF);

        ad7175.contConvReadData(&data);
        data_count++;
        data_values[data.status.active_channel] = data.value;

        if (data_count % NUMBER_ANALOG_CHANNELS == 0) {
            for (int i = 0; i < NUMBER_ANALOG_CHANNELS; i++) {
                float voltage_value = ((double)(data_values[i]) * 5) / (std::pow(2, 24));
                Serial.print(voltage_value, 5);
                Serial.print(" ");
            }

            Serial.println();
        }
        delay(20);
    }
}

uint32_t interrupt_counts[6] = {0, 0, 0, 0, 0, 0};

void Test::readDigitalChannels(void) {
    pinMode(D1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D1_PIN), D4InterruptFn, RISING);

    pinMode(D2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D2_PIN), D5InterruptFn, RISING);

    pinMode(D3_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D3_PIN), D6InterruptFn, RISING);

    pinMode(D4_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D4_PIN), D4InterruptFn, RISING);

    pinMode(D5_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D5_PIN), D5InterruptFn, RISING);

    pinMode(D6_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(D6_PIN), D6InterruptFn, RISING);

    for (;;) {
        delayMicroseconds(1000);

        Serial.println("Interrupts per second: ");
        Serial.print("Channel D1 ");
        Serial.println(interrupt_counts[0]);
        Serial.print("Channel D2 ");
        Serial.println(interrupt_counts[1]);
        Serial.print("Channel D3 ");
        Serial.println(interrupt_counts[2]);
        Serial.print("Channel D4 ");
        Serial.println(interrupt_counts[3]);
        Serial.print("Channel D5 ");
        Serial.println(interrupt_counts[4]);
        Serial.print("Channel D6 ");
        Serial.println(interrupt_counts[5]);

        for (int i = 0; i < 6; i++) {
            interrupt_counts[i] = 0;
        }
    }
}

void Test::D1InterruptFn(void) {
    interrupt_counts[0]++;
}

void Test::D2InterruptFn(void) {
    interrupt_counts[1]++;
}

void Test::D3InterruptFn(void) {
    interrupt_counts[2]++;
}

void Test::D4InterruptFn(void) {
    interrupt_counts[3]++;
}

void Test::D5InterruptFn(void) {
    interrupt_counts[4]++;
}

void Test::D6InterruptFn(void) {
    interrupt_counts[5]++;
}