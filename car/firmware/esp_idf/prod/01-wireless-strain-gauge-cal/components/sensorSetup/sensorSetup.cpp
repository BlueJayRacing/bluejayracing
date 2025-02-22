#include <sensorSetup.hpp>
#include <esp_log.h>

#define NUM_MEASUREMENTS 100
#define ERROR_THRESHOLD 0.005

static const char* TAG = "sensorSetup";

sensorSetup::sensorSetup() {};

esp_err_t sensorSetup::init(ads1120_init_param_t adc_params, ad5626_init_param_t dac_params)
{
    esp_err_t ret = adc_.init(adc_params);
    if (ret) {
        return ret;
    }

    ret = dac_.init(dac_params);
    if (ret) {
        return ret;
    }

    ads1120_regs_t regs;
    memset(&regs, 0, sizeof(ads1120_regs_t));

    regs.conv_mode = CONTINUOUS;
    regs.op_mode   = NORMAL;
    regs.channels  = AIN1_AVSS;
    regs.data_rate = 2;
    regs.volt_refs = REFP0_REFN0;
    regs.gain = GAIN_1;

    ret = adc_.configure(regs);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t sensorSetup::zero(void) {
    float average_volt_error;
    sensor_measurement_t measurements[NUM_MEASUREMENTS];

    do {
        // Calculate the average voltage error
        float sum_volt_errors = 0;

        for (int i = 0; i < NUM_MEASUREMENTS; i++) {
            measure(&(measurements[i]));
            sum_volt_errors += (measurements[i].voltage - 2.5);
        }

        average_volt_error = sum_volt_errors / NUM_MEASUREMENTS;

        int16_t dac_shift = - average_volt_error / measurements[0].gain * 1000;

        ESP_LOGI(TAG, "\nAverage Volt Error: %f", average_volt_error);
        ESP_LOGI(TAG, "Calculated DAC Shift: %d", dac_shift);
        ESP_LOGI(TAG, "Setting DAC Bias: %d", measurements[0].dac_bias + dac_shift);

        dac_.setLevel(measurements[0].dac_bias + dac_shift);
    } while (std::abs(average_volt_error) > ERROR_THRESHOLD);

    return ESP_OK;
}

esp_err_t sensorSetup::setGain(ads1120_gain_t gain)
{
    ads1120_regs_t adc_regs;
    adc_.getRegs(&adc_regs);

    adc_regs.gain = gain;

    esp_err_t ret = adc_.configure(adc_regs);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t sensorSetup::measure(sensor_measurement_t* t_meas)
{
    // Spin until the ADC has data ready
    while (!adc_.isDataReady()) {
    }

    esp_err_t ret = adc_.readADC(&(t_meas->adc_value));
    if (ret) {
        return ret;
    }

    t_meas->dac_bias = dac_.getLevel();

    ads1120_regs_t regs;
    adc_.getRegs(&regs);

    switch (regs.gain) {
    case (GAIN_1):
    t_meas->gain = 1;
        break;
    case (GAIN_2):
    t_meas->gain = 2;
        break;
    case (GAIN_4):
    t_meas->gain = 4;
        break;
    case (GAIN_8):
    t_meas->gain = 8;
        break;
    case (GAIN_16):
    t_meas->gain = 16;
        break;
    case (GAIN_32):
    t_meas->gain = 32;
        break;
    case (GAIN_64):
    t_meas->gain = 64;
        break;
    case (GAIN_128):
    t_meas->gain = 128;
        break;
    }

    t_meas->voltage = t_meas->gain * 5.001 * t_meas->adc_value / ((1 << 15) - 1);

    return ESP_OK;
}