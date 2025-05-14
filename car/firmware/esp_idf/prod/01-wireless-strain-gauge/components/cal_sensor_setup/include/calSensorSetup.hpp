#pragma once
#ifndef _CAL_SENSOR_SETUP_HPP_
#define _CAL_SENSOR_SETUP_HPP_

#include <ad5626.hpp>
#include <ads1120.hpp>
#include <esp_event.h>
#include <esp_system.h>

typedef struct cal_measurement {
  float voltage;
  uint8_t gain;
  int16_t adc_value;
  uint16_t dac_bias;
} cal_measurement_t;

class calSensorSetup {
  public:
    calSensorSetup();
    esp_err_t init(ads1120_init_param_t adc_params, ad5626_init_param_t dac_params);
    esp_err_t zero(void);
    esp_err_t setGain(ads1120_gain_t gain);
    esp_err_t measure(cal_measurement_t* measurement);

  private:
    AD5626 dac_;
    ADS1120 adc_;
};

#endif