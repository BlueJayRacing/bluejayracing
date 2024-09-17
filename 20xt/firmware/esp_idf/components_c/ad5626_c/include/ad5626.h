#pragma once
#ifndef _AD5626_H_
#define _AD5626_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"

esp_err_t ad5626_spi_init(int8_t cs_pin, gpio_num_t ldac_pin, gpio_num_t clr_pin, spi_host_device_t spi_host);

esp_err_t ad5626_set_level(const uint16_t new_dac_level);

esp_err_t ad5626_clear_level_reg(void);

#ifdef __cplusplus
}
#endif

#endif