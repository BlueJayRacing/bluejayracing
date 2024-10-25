#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#include "ad5626.h"

#define AD5626_MAX_TICKS_WAIT 10

spi_device_handle_t ad5626_spi_dev;
gpio_num_t ad5626_ldac_pin;
gpio_num_t ad5626_clr_pin;

esp_err_t ad5626_spi_init(int8_t cs_pin, gpio_num_t ldac_pin, gpio_num_t clr_pin, spi_host_device_t spi_host)
{
    spi_device_interface_config_t ad5626_cfg;
    memset(&ad5626_cfg, 0, sizeof(spi_device_interface_config_t));

    ad5626_cfg.mode           = 3;
    ad5626_cfg.clock_speed_hz = 1000000;
    ad5626_cfg.spics_io_num   = cs_pin;
    ad5626_cfg.flags          = SPI_DEVICE_HALFDUPLEX;
    ad5626_cfg.queue_size     = 1;
    ad5626_cfg.pre_cb         = NULL;
    ad5626_cfg.post_cb        = NULL;

    ad5626_ldac_pin = ldac_pin;
    ad5626_clr_pin  = clr_pin;

    esp_err_t ret = gpio_set_direction(ad5626_ldac_pin, GPIO_MODE_OUTPUT);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(ad5626_ldac_pin, 1);
    if (ret) {
        return ret;
    }

    if (ad5626_clr_pin >= 0) {
        ret = gpio_set_direction(ad5626_clr_pin, GPIO_MODE_OUTPUT);
        if (ret) {
            return ret;
        }

        ret = gpio_set_level(ad5626_clr_pin, 1);
        if (ret) {
            return ret;
        }
    }

    return spi_bus_add_device(spi_host, &ad5626_cfg, &ad5626_spi_dev);
}

esp_err_t ad5626_set_level(const uint16_t new_dac_level)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    uint8_t new_level[2];
    new_level[0] = ((new_dac_level & 0x0F00) / 16) + ((new_dac_level & 0x00F0) / 16);
    new_level[1] = ((new_dac_level & 0x000F) * 16);
    t.tx_buffer  = &new_level;
    t.length     = 12;

    esp_err_t ret = spi_device_polling_transmit(ad5626_spi_dev, &t);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(ad5626_ldac_pin, 0);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(ad5626_ldac_pin, 1);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t ad5626_clear_level_reg(void)
{
    if (ad5626_clr_pin < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = gpio_set_level(ad5626_clr_pin, 0);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(ad5626_clr_pin, 0);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}