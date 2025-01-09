#pragma once
#ifndef _SENSOR_SETUP_HPP_
#define _SENSOR_SETUP_HPP_

#include <ads1120.hpp>
#include <ad5626.hpp>
#include <esp_event.h>
#include <esp_system.h>

class sensorSetup {
  public:
    sensorSetup();
    esp_err_t configureADC();
    esp_err_t calibrate();
    const esp_err_t measure(const float* measurement);
  private:
    AD5626 dac;
    ADS1120 adc;
};

#endif