#ifndef _AD5689_HPP_
#define _AD5689_HPP_

#include <Arduino.h>
#include <SPI.h>

typedef struct ad5689_init_params {
    int8_t cs_pin;
    int8_t ldac_pin; 
    int8_t clr_pin;
    SPIClass* spi_host;
} ad5689_init_param_t;

typedef enum ad5689_command {
  WRITE_IN_REG = 0x01,
  UPDATE_DAC_REG = 0x02,
  WRITE_DAC_REG = 0x03,
  POWER = 0x04,
  LDAC_MASK = 0x05,
  RESET = 0x06,
  REF_SETUP_REG = 0x07,
  SETUP_DCEN_REG = 0x08,
  SETUP_READBACK_REG = 0x0A,
  NO_OP_DC_MODE = 0x0F
} ad5689_command_t;

typedef enum ad5689_channel {
  A = 0x01,
  B = 0x08,
  BOTH = 0x09
} ad5689_channel_t;

class AD5689 {
  public:
    AD5689() : spi_host_(&SPI), spi_settings_(2000000, MSBFIRST, SPI_MODE1) {};
    void init(const ad5689_init_param_t init_params);
    void transfer(const ad5689_command_t command, const ad5689_channel_t t_chan_mode, const std::array<uint8_t, 2>& t_data);
    void setLevel(const ad5689_channel_t t_chan_mode, const uint16_t t_new_dac_level);

  private:
    ad5689_init_param_t bus_params_;
    SPISettings spi_settings_;
    SPIClass* spi_host_;
};

#endif