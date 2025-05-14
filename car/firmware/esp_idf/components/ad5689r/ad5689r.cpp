#include <string.h>

#include "ad5689r.hpp"

AD5689R::AD5689R()
{
    current_level_.chan_a_level = 0;
    current_level_.chan_b_level = 0;
}

/*******************************************************************************
 * @brief Initializes the AD5689R.
 *
 * @param init_params - pins and SPI host
 *******************************************************************************/
esp_err_t AD5689R::init(ad5689r_params_t t_init_params)
{
    esp_err_t ret;

    params_ = t_init_params;

    spi_device_interface_config_t ad5689r_cfg;
    memset(&ad5689r_cfg, 0, sizeof(spi_device_interface_config_t));

    ad5689r_cfg.mode           = 1;
    ad5689r_cfg.clock_speed_hz = 1000000;
    ad5689r_cfg.spics_io_num   = params_.cs_pin;
    ad5689r_cfg.flags          = SPI_DEVICE_HALFDUPLEX;
    ad5689r_cfg.queue_size     = 1;
    ad5689r_cfg.pre_cb         = NULL;
    ad5689r_cfg.post_cb        = NULL;

    ret = gpio_set_direction(params_.ldac_pin, GPIO_MODE_OUTPUT);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(params_.ldac_pin, 1);
    if (ret) {
        return ret;
    }

    if (params_.clr_pin >= 0) {
        ret = gpio_set_direction(params_.clr_pin, GPIO_MODE_OUTPUT);
        if (ret) {
            return ret;
        }

        ret = gpio_set_level(params_.clr_pin, 1);
        if (ret) {
            return ret;
        }
    }

    if (params_.gain_pin >= 0) {
        ret = gpio_set_direction(params_.gain_pin, GPIO_MODE_OUTPUT);
        if (ret) {
            return ret;
        }

        ret = gpio_set_level(params_.gain_pin, 1);
        if (ret) {
            return ret;
        }
    }

    ret = spi_bus_add_device(params_.spi_host, &ad5689r_cfg, &(spi_dev_));
    if (ret) {
        return ret;
    }

    return setLevel(BOTH, 0);
}

/*******************************************************************************
 * @brief Sends an SPI transaction to the AD5689R device.
 *
 * @param t_command   - The type of command to send to the AD5689R.
 * @param t_chan_mode - The desired DAC channel (A, B, BOTH).
 * @param t_data      - The data to be sent to the AD5689R device.
 *******************************************************************************/
esp_err_t AD5689R::transfer(const ad5689r_command_t t_command, const ad5689r_chan_t t_chan_mode,
                            const std::array<uint8_t, 2>& t_data)
{
    esp_err_t ret;

    std::array<uint8_t, 3> buf;
    buf[0] = (t_command << 4) | t_chan_mode;
    buf[1] = t_data[0];
    buf[2] = t_data[1];

    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));

    t.tx_buffer  = buf.data();
    t.length     = buf.size() * 8;

    ret = spi_device_queue_trans(spi_dev_, &t, portMAX_DELAY); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    spi_transaction_t* ret_t;

    ret = spi_device_get_trans_result(spi_dev_, &ret_t, portMAX_DELAY); // Transmit!
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_level(params_.ldac_pin, 0);
    if (ret) {
        return ret;
    }

    ret = gpio_set_level(params_.ldac_pin, 1);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}

/*******************************************************************************
 * @brief Sets the level for one of the DAC channels on the AD5689R.
 *
 * @param t_chan_id  - The desired DAC channel (A, B, BOTH).
 * @param t_new_dac_level - The new value for the DAC channel.
 *******************************************************************************/
esp_err_t AD5689R::setLevel(const ad5689r_chan_t t_chan, const uint16_t t_new_dac_level)
{
    esp_err_t ret;

    if (t_new_dac_level > AD5689R::MAX_LEVEL_VALUE) {
        return ESP_ERR_INVALID_ARG;
    }

    std::array<uint8_t, 2> t_data;
    t_data[0] = (t_new_dac_level & 0xFF00) >> 8;
    t_data[1] = (t_new_dac_level & 0x00FF);
    ret = transfer(WRITE_DAC_REG, t_chan, t_data);

    if (ret) {
        return ret;
    }

    switch (t_chan) {
    case A:
        current_level_.chan_a_level = t_new_dac_level;
        break;
    case B:
        current_level_.chan_b_level = t_new_dac_level;
        break;
    case BOTH:
        current_level_.chan_a_level = t_new_dac_level;
        current_level_.chan_b_level = t_new_dac_level;
        break;
    }

    return ESP_OK;
}

esp_err_t AD5689R::clearLevel(void) {
    esp_err_t ret;

    if (params_.clr_pin >= 0) { // Use clear pin if possible
        ret = gpio_set_level(params_.clr_pin, 0);
        if (ret) {
            return ret;
        }
    
        ret = gpio_set_level(params_.clr_pin, 1);
        if (ret) {
            return ret;
        }
    } else {// Use normal set level call if clean pin unavailable
        std::array<uint8_t, 2> t_data;
        t_data[0] = 0;
        t_data[1] = 0;

        ret = transfer(WRITE_DAC_REG, BOTH, t_data);
        if (ret) {
            return ret;
        }
    }

    current_level_.chan_a_level = 0;
    current_level_.chan_b_level = 0;

    return ESP_OK;
}

esp_err_t AD5689R::setGain(bool is_high) {
    esp_err_t ret;

    if (params_.gain_pin < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = gpio_set_level(params_.gain_pin, is_high);
    if (ret) {
        return ret;
    }

    return ESP_OK;
}
