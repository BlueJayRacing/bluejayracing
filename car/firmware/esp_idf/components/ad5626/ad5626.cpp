#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdio.h>
#include <string.h>
#include <array>

#include "ad5626.hpp"

AD5626::AD5626() {current_level_ = 0; }

/*******************************************************************************
 * @brief Initializes the AD5626.
 *
 * @param t_init_param - Initialization parameters for AD5626. Note that the
 *                       clear pin is optional and can be set to GPIO_NUM_NC.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t AD5626::init(const ad5626_init_param_t t_init_param)
{
    ldac_pin_ = t_init_param.ldac_pin;
    clr_pin_  = t_init_param.clr_pin;

    spi_device_interface_config_t ad5626_cfg;
    memset(&ad5626_cfg, 0, sizeof(spi_device_interface_config_t));

    ad5626_cfg.mode           = 3;
    ad5626_cfg.clock_speed_hz = 1000000;
    ad5626_cfg.spics_io_num   = t_init_param.cs_pin;
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

    ret = spi_bus_add_device(t_init_param.spi_host, &ad5626_cfg, &(spi_dev_));
    if (ret) {
        return ret;
    }

    return setLevel(0);
}

/*******************************************************************************
 * @brief Sets and loads the DAC with a 12-bit value.
 *
 * @param t_dac_new_level - The 12-bit value to be loaded onto the DAC register.
 *                          The max value is 4095.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t AD5626::setLevel(const uint16_t t_new_dac_level)
{
    if (t_new_dac_level > AD5626::MAX_LEVEL_VALUE) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));
    std::array<uint8_t, 2> new_level;

    new_level[0] = ((t_new_dac_level & 0x0F00) / 16) + ((t_new_dac_level & 0x00F0) / 16);
    new_level[1] = ((t_new_dac_level & 0x000F) * 16);
    t.tx_buffer  = new_level.data();
    t.length     = 12;

    esp_err_t ret = spi_device_queue_trans(spi_dev_, &t, portMAX_DELAY); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    spi_transaction_t* ret_t;

    ret = spi_device_get_trans_result(spi_dev_, &ret_t, portMAX_DELAY); // Transmit!
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

    current_level_ = t_new_dac_level;

    return ESP_OK;
}

/*******************************************************************************
 * @brief Resets the DAC level to 0.
 *
 * @return Returns 0 for success or ESP_ERR_INVALID_STATE if the pin has not been
 *         set.
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

    current_level_ = 0;

    return ESP_OK;
}

/*******************************************************************************
 * @brief Gets the current DAC level.
 *
 * @return Returns the current DAC level.
 *******************************************************************************/
uint16_t AD5626::getLevel(void) {
    return current_level_;
}