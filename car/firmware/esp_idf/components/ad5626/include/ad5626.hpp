#pragma once
#ifndef _AD5626_HPP_
#define _AD5626_HPP_

#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

class AD5626 {
    public:
        AD5626();
        esp_err_t init(const gpio_num_t t_cs_pin, const gpio_num_t t_ldac_pin, const gpio_num_t t_clr_pin, const spi_host_device_t t_spi_host);
        esp_err_t setLevel(const uint16_t t_new_dac_level);
        esp_err_t clearLevel(void);
    private:
        spi_device_handle_t spi_dev_;
        gpio_num_t ldac_pin_;
        gpio_num_t clr_pin_;
};

#endif