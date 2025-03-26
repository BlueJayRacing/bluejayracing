#include <test.hpp>

#define ADC_CS_PIN 10

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
    ad717x_filter_config_t filt_con     = {false, false, SPS27_DB47_MS36P7, SINC5_SINC1, SPS_25000};
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

                Serial.print(voltage_value);
                Serial.print(" ");
            }

            Serial.println();
        }
    }
}

void Test::continuousChannelReadStats(void)
{
    SPI.begin();

    // Set up the AD7175 parameters
    ad717x_init_param_t ad7175_params;

    // Configure analog input channels
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

    // Set channel configuration
    ad717x_channel_config_t chan_config = {true, false, false, AVDD_AVSS};
    // Set filter configuration
    ad717x_filter_config_t filt_con = {false, false, SPS27_DB47_MS36P7, SINC5_SINC1, SPS_250000};
    // Combine into one setup
    ad717x_setup_t setup = {1, chan_config, filt_con};

    // Configure device-specific parameters
    ad7175_params.active_device   = ID_AD7175_8;
    ad7175_params.ref_en          = true;
    ad7175_params.mode            = CONTINUOUS;
    ad7175_params.stat_on_read_en = true;

    // Build channel mapping for each analog channel
    for (int i = 0; i < NUMBER_ANALOG_CHANNELS; i++) {
        ad717x_channel_map_t chan_map = {true, 0, chan_inputs[i]};
        ad7175_params.chan_map.push_back(chan_map);
    }
    ad7175_params.setups.push_back(setup);

    // Initialize the AD7175 device
    ad7175.init(ad7175_params, &SPI, ADC_CS_PIN);

    // Variables to accumulate statistics
    const unsigned long statInterval = 2000000UL; // 2 seconds in microseconds
    unsigned long intervalStart = micros();
    unsigned long previousTime = intervalStart;
    unsigned long totalChannelTime = 0;
    unsigned long readCount = 0;
    ad717x_data_t data;

    int counter = 0;
    uint32_t tempPrev = micros();

    while (true) {
        // Wait until data is ready and perform a read
        counter++;
        ad7175.waitForReady(0xFFFFFFFF);
        ad7175.contConvReadData(&data);
        
        // Compute elapsed time for this read
        unsigned long currentTime = micros();
        unsigned long readTime = currentTime - previousTime;
        totalChannelTime += readTime;
        previousTime = currentTime;
        readCount++;

        // Check if the interval has elapsed
        if (currentTime - intervalStart >= statInterval) {
            // Calculate average channel read time (in microseconds)
            float avgChannelTime = totalChannelTime / (float)readCount;
            // Calculate the real read rate (reads per second)
            float realRate = 1000000.0f / avgChannelTime;

            Serial.print("Average channel read time (us): ");
            Serial.println(avgChannelTime);
            Serial.print("Real channel read rate (reads/sec): ");
            Serial.println((double)counter/(currentTime-tempPrev)*1000000.0f);
            Serial.println("---------------------------");

            // Reset counters for the next interval
            intervalStart = currentTime;
            totalChannelTime = 0;
            readCount = 0;

            counter = 0;
            tempPrev = currentTime;
        }
    }
}


void Test::readDigitalChannels(void) {
    
}