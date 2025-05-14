#include <driveSensorSetup.hpp>
#include <esp_log.h>

#define ZEROING_MAX_TRIES 10
#define NUM_MEASUREMENTS  100
#define ERROR_THRESHOLD   0.003

static const char* TAG = "driveSensorSetup";

int min_(int a, int b);

int max_(int a, int b);

driveSensorSetup::driveSensorSetup() {};

/*******************************************************************************
 * @brief Initializes the sensorSetup.
 *
 * @param adc_params - The pin confiugration parameters for the ADC.
 * @param dac_params - The pin configuration parameters for the DAC.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t driveSensorSetup::init(ads1120_init_param_t adc_params, ad5626_init_param_t dac_params)
{
    esp_err_t ret = adc_.init(adc_params);
    if (ret) {
        return ret;
    }

    ret = dac_.init(dac_params);
    if (ret) {
        return ret;
    }

    drive_cfg_t init_cfg = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::STRAIN_GAUGE};
    ret                  = configure(init_cfg);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t driveSensorSetup::measure(bool wait_ready, drive_measurement_t* t_meas)
{
    if (t_meas == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Spin until the ADC has data ready
    if (wait_ready) {
        while (!adc_.isDataReady()) {
        }
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

/*******************************************************************************
 * @brief Zeroes the sensorSetup a maximum of ZEROING_MAX_TRIES times before
 *        failing.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t driveSensorSetup::zero(void)
{
    esp_err_t ret;
    bool successful_zero = false;
    drive_measurement_t measurements[NUM_MEASUREMENTS];

    // Set ADC into Zeroing Mode
    drive_cfg_t old_cfg  = cfg_;
    drive_cfg_t zero_cfg = {drive_cfg_t::ZEROING_MODE, drive_cfg_t::STRAIN_GAUGE};
    ret                  = configure(zero_cfg);
    if (ret) {
        return ret;
    }

    for (uint8_t i = 0; i < ZEROING_MAX_TRIES; i++) {
        // Calculate the average voltage error
        float sum_volt_errors = 0;

        for (int i = 0; i < NUM_MEASUREMENTS; i++) {
            ret = measure(true, &(measurements[i]));
            if (ret) {
                vTaskDelay(10);
                continue;
            }

            sum_volt_errors += (measurements[i].voltage - 2.5);
        }

        float average_volt_error = sum_volt_errors / NUM_MEASUREMENTS;

        if (std::abs(average_volt_error) < ERROR_THRESHOLD) {
            successful_zero = true;
            break;
        } else if (i == ZEROING_MAX_TRIES) {
            successful_zero = false;
            break;
        }

        int16_t new_dac_bias = measurements[0].dac_bias - average_volt_error / measurements[0].gain * 1000;

        ESP_LOGI(TAG, "Previous DAC Bias: %d", measurements[0].dac_bias);
        ESP_LOGI(TAG, "Calculated Average Voltage Error: %f", average_volt_error);
        ESP_LOGI(TAG, "Calculated New DAC Bias: %d", new_dac_bias);

        new_dac_bias = max_(new_dac_bias, 0);
        new_dac_bias = min_(new_dac_bias, AD5626::MAX_LEVEL_VALUE);

        ESP_LOGI(TAG, "New DAC Bias: %d", new_dac_bias);

        dac_.setLevel(new_dac_bias);
        vTaskDelay(1);
    }

    drive_measurement_t last_measurement;
    ret = measure(true, &last_measurement);
    if (ret) {
        return ret;
    }

    ESP_LOGI(TAG, "Final DAC Bias: %d", last_measurement.dac_bias);
    ESP_LOGI(TAG, "Final Voltage: %f", last_measurement.voltage);

    // Set ADC into Measuring Mode
    ret = configure(old_cfg);
    if (ret) {
        return ret;
    }

    if (successful_zero) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t driveSensorSetup::configure(drive_cfg_t new_cfg)
{
    esp_err_t ret;
    ads1120_regs_t regs;
    memset(&regs, 0, sizeof(ads1120_regs_t));

    switch (new_cfg.channel) {
    case drive_cfg_t::STRAIN_GAUGE:
        regs.channels = AIN1_AVSS;
        break;
    case drive_cfg_t::EXCITATION:
        regs.channels = AIN0_AVSS;
        break;
    case drive_cfg_t::DAC_BIAS:
        regs.channels = AIN2_AVSS;
        break;
    }

    switch (new_cfg.mode) {
    case drive_cfg_t::ZEROING_MODE:
        regs.conv_mode = CONTINUOUS;
        regs.op_mode   = NORMAL;
        regs.data_rate = 2;
        regs.volt_refs = REFP0_REFN0;
        regs.gain      = GAIN_1;
        break;
    case drive_cfg_t::MEASURING_MODE:
        regs.conv_mode = CONTINUOUS;
        regs.op_mode   = TURBO;
        regs.data_rate = 6;
        regs.volt_refs = REFP0_REFN0;
        regs.gain      = GAIN_1;
        break;
    }

    ret = adc_.configure(regs);
    if (ret) {
        return ret;
    }

    cfg_ = new_cfg;

    return ESP_OK;
}

int min_(int a, int b) { return (a < b) ? a : b; }

int max_(int a, int b) { return (a > b) ? a : b; }