#pragma once
#ifndef _SENSOR_SETUP_HPP_
#define _SENSOR_SETUP_HPP_

#include <ad5626.hpp>
#include <ads1120.hpp>
#include <esp_event.h>
#include <esp_system.h>


class sensorSetup {
  public:
    sensorSetup();
    esp_err_t init(ads1120_init_param_t adc_params, ad5626_init_param_t dac_params);
    const esp_err_t measure(const float* measurement);
    // Zreo first
    // Change gain as necessary by feeding in loads

  private:
    AD5626 dac;
    ADS1120 adc;
};

#endif