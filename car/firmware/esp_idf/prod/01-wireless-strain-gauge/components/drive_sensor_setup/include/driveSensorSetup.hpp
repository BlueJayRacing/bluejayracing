#pragma once
#ifndef _DRIVE_SENSOR_SETUP_HPP_
#define _DRIVE_SENSOR_SETUP_HPP_

#include <ad5626.hpp>
#include <ads1120.hpp>
#include <esp_event.h>
#include <esp_system.h>

typedef struct drive_cfg
{ 
  enum mode_t { ZEROING_MODE, MEASURING_MODE } mode;
  enum channel_t {STRAIN_GAUGE, EXCITATION, DAC_BIAS } channel;
} drive_cfg_t;

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
    esp_err_t configure(drive_cfg_t new_cfg);
    esp_err_t measure(bool wait_ready, drive_measurement_t* measurement);

  private:
    AD5626 dac_;
    ADS1120 adc_;
    drive_cfg_t cfg_;
};

#endif