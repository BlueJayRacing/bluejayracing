#pragma once
#ifndef _DRIVE_SENSOR_SETUP_HPP_
#define _DRIVE_SENSOR_SETUP_HPP_

#include <ad5626.hpp>
#include <ads1120.hpp>
#include <esp_event.h>
#include <esp_system.h>

typedef struct drive_cal_param {
  ads1120_gain_t gain;
} drive_cal_param_t;

typedef enum drive_sensor_mode
{
    ZEROING_MODE,
    MEASURING_MODE,
} drive_sensor_mode_t;

typedef struct drive_measurement {
    float voltage;
    uint8_t gain;
    int16_t adc_value;
    uint16_t dac_bias;
} drive_measurement_t;

class driveSensorSetup {
  public:
    driveSensorSetup();
    esp_err_t init(ads1120_init_param_t adc_params, ad5626_init_param_t dac_params);
    esp_err_t zero(void);
    esp_err_t setCalParams(drive_cal_param_t cal_params);
    esp_err_t measure(drive_measurement_t* measurement, bool wait_ready);

  private:
    esp_err_t setMode(drive_sensor_mode_t mode);

  private:
    AD5626 dac_;
    ADS1120 adc_;
    drive_cal_param_t params_;
    drive_sensor_mode_t mode_;
};

#endif