#pragma once
#ifndef _AD5626_HPP_
#define _AD5626_HPP_

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_system.h>

// The Clear Pin is optional
typedef struct ad5626_init_param {
  gpio_num_t cs_pin;
  gpio_num_t ldac_pin;
  gpio_num_t clr_pin;
  spi_host_device_t spi_host;
} ad5626_init_param_t;

class AD5626 {
  public:
    AD5626();
    esp_err_t init(const ad5626_init_param_t t_init_param);
    esp_err_t setLevel(const uint16_t t_new_dac_level) const;
    esp_err_t clearLevel(void) const;

  public:
    static const int MAX_LEVEL_VALUE = 4095;

  private:
    spi_device_handle_t spi_dev_;
    gpio_num_t ldac_pin_;
    gpio_num_t clr_pin_;
};

#endif