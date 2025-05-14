#pragma once
#ifndef _AD5689R_HPP_
#define _AD5689R_HPP_

#include <array>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_system.h>

// The Clear Pin is optional
typedef struct ad5689r_params {
    gpio_num_t cs_pin;
    gpio_num_t ldac_pin;
    gpio_num_t clr_pin;  // Optional
    gpio_num_t gain_pin; // Optional - library assumes pin is pulled high
    spi_host_device_t spi_host;
} ad5689r_params_t;

typedef enum ad5689r_command
{
    WRITE_IN_REG       = 0x01,
    UPDATE_DAC_REG     = 0x02,
    WRITE_DAC_REG      = 0x03,
    POWER              = 0x04,
    LDAC_MASK          = 0x05,
    RESET              = 0x06,
    REF_SETUP_REG      = 0x07,
    SETUP_DCEN_REG     = 0x08,
    SETUP_READBACK_REG = 0x0A,
    NO_OP_DC_MODE      = 0x0F
} ad5689r_command_t;

typedef enum ad5689r_chan
{
    A    = 0x01,
    B    = 0x08,
    BOTH = 0x09
} ad5689r_chan_t;

typedef struct ad5689r_level {
    uint16_t chan_a_level;
    uint16_t chan_b_level;
} ad5689r_level_t;

class AD5689R {
  public:
    AD5689R();
    esp_err_t init(const ad5689r_params_t t_init_params);
    esp_err_t transfer(const ad5689r_command_t command, const ad5689r_chan_t t_chan,
      const std::array<uint8_t, 2>& t_data);
    esp_err_t setLevel(const ad5689r_chan_t t_chan, const uint16_t t_new_dac_level);
    esp_err_t clearLevel(void);
    esp_err_t setGain(bool is_double);
    ad5689r_level_t getLevel(ad5689r_chan_t t_chan);

  public:
    static const uint16_t MAX_LEVEL_VALUE = 0xFFFF;

  private:
    spi_device_handle_t spi_dev_;
    ad5689r_params_t params_;
    ad5689r_level_t current_level_;
};

#endif