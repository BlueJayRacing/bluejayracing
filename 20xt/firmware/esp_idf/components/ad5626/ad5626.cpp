#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <string.h>

#include "ad5626.hpp"

AD5626::AD5626()
{
}

esp_err_t AD5626::init(const gpio_num_t t_cs_pin, const gpio_num_t t_ldac_pin, const gpio_num_t t_clr_pin, const spi_host_device_t t_spi_host)
{
    m_ldac_pin = t_ldac_pin;
    m_clr_pin = t_clr_pin;

    spi_device_interface_config_t ad5626_cfg;
    memset(&ad5626_cfg, 0, sizeof(spi_device_interface_config_t));

    ad5626_cfg.mode = 3;
    ad5626_cfg.clock_speed_hz = 1000000;
    ad5626_cfg.spics_io_num = t_cs_pin;
    ad5626_cfg.flags = SPI_DEVICE_HALFDUPLEX;
    ad5626_cfg.queue_size = 1;
    ad5626_cfg.pre_cb = NULL;
    ad5626_cfg.post_cb = NULL;

    esp_err_t ret = gpio_set_direction(m_ldac_pin, GPIO_MODE_OUTPUT);
    if (ret)
    {
        return ret;
    }

    ret = gpio_set_level(m_ldac_pin, 1);
    if (ret)
    {
        return ret;
    }

    if (m_clr_pin >= 0)
    {
        ret = gpio_set_direction(m_clr_pin, GPIO_MODE_OUTPUT);
        if (ret)
        {
            return ret;
        }

        ret = gpio_set_level(m_clr_pin, 1);
        if (ret)
        {
            return ret;
        }
    }

    return spi_bus_add_device(t_spi_host, &ad5626_cfg, &(m_spi_dev));
}

esp_err_t AD5626::setLevel(const uint16_t t_new_dac_level)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));
    uint8_t new_level[2];
    new_level[0] = ((t_new_dac_level & 0x0F00) / 16) + ((t_new_dac_level & 0x00F0) / 16);
    new_level[1] = ((t_new_dac_level & 0x000F) * 16);
    t.tx_buffer = &new_level;
    t.length = 12;

    esp_err_t ret = spi_device_polling_transmit(m_spi_dev, &t); // Transmit!
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = gpio_set_level(m_ldac_pin, 0);
    if (ret)
    {
        return ret;
    }

    ret = gpio_set_level(m_ldac_pin, 1);
    if (ret)
    {
        return ret;
    }

    return ESP_OK;
}

esp_err_t AD5626::clearLevel(void)
{
    if (m_clr_pin < 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = gpio_set_level(m_clr_pin, 0);
    if (ret)
    {
        return ret;
    }

    ret = gpio_set_level(m_clr_pin, 0);
    if (ret)
    {
        return ret;
    }

    return ESP_OK; 
}