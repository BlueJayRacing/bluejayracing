#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>

#include "ad5626.hpp"

AD5626::AD5626() {}

/*******************************************************************************
 * @brief Initializes the AD5626.
 *
 * @param t_cs_pin   - The chip select
 *pin.
 * @param t_ldac_pin - The load to
 *register pin.
 * @param t_clr_pin  - The clear pin. (set
 *to <0 if not used)
 * @param t_spi_host - The SPI host/bus
 *that the device is on.
 *
 * @return Returns 0 for success or
 *negative error code.
 *******************************************************************************/
esp_err_t AD5626::init(const gpio_num_t t_cs_pin, const gpio_num_t t_ldac_pin, const gpio_num_t t_clr_pin,
                       const spi_host_device_t t_spi_host)
{
    ldac_pin_ = t_ldac_pin;
    clr_pin_  = t_clr_pin;

    spi_device_interface_config_t ad5626_cfg;
    memset(&ad5626_cfg, 0, sizeof(spi_device_interface_config_t));

    ad5626_cfg.mode           = 3;
    ad5626_cfg.clock_speed_hz = 1000000;
    ad5626_cfg.spics_io_num   = t_cs_pin;
    ad5626_cfg.flags          = SPI_DEVICE_HALFDUPLEX;
    ad5626_cfg.queue_size     = 1;
    ad5626_cfg.pre_cb         = NULL;
    ad5626_cfg.post_cb        = NULL;

    esp_err_t ret = gpio_set_direction(ldac_pin_, GPIO_MODE_OUTPUT);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(ldac_pin_, 1);
    if (ret) {
        return ret;
    }

    if (clr_pin_ >= 0) {
        ret = gpio_set_direction(clr_pin_, GPIO_MODE_OUTPUT);
        if (ret) {
            return ret;
        }

        ret = gpio_set_level(clr_pin_, 1);
        if (ret) {
            return ret;
        }
    }

    return spi_bus_add_device(t_spi_host, &ad5626_cfg, &(spi_dev_));
}

/*******************************************************************************
 * @brief Sets and loads the DAC with a
 *12-bit value.
 *
 * @param t_dac_new_level - The 12-bit
 *value to be loaded onto the DAC
 *register.
 *
 * @return Returns 0 for success or
 *negative error code.
 *******************************************************************************/
esp_err_t AD5626::setLevel(const uint16_t t_new_dac_level)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));
    uint8_t new_level[2];
    new_level[0] = ((t_new_dac_level & 0x0F00) / 16) + ((t_new_dac_level & 0x00F0) / 16);
    new_level[1] = ((t_new_dac_level & 0x000F) * 16);
    t.tx_buffer  = &new_level;
    t.length     = 12;

    esp_err_t ret = spi_device_polling_transmit(spi_dev_, &t); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_level(ldac_pin_, 0);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(ldac_pin_, 1);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

/*******************************************************************************
 * @brief Resets the DAC level to 0.
 *
 * @return Returns 0 for success or
 *negative error code, specfically
 * ESP_ERR_INVALID_STATE if the pin has
 *not been set.
 *******************************************************************************/
esp_err_t AD5626::clearLevel(void)
{
    if (clr_pin_ < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = gpio_set_level(clr_pin_, 0);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(clr_pin_, 0);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}