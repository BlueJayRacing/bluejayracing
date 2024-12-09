#ifndef _AD5676_HPP_
#define _AD5676_HPP_

#include <Arduino.h>
#include <SPI.h>

typedef enum ad5676_command
{
    AD5676_COM_NO_OP                   = 0x00,
    AD5676_COM_INP_REG_WRITE           = 0x01,
    AD5676_COM_DAC_REG_UPDATE          = 0x02,
    AD5676_COM_DAC_REG_WRITE_UPDATE    = 0x03,
    AD5676_COM_POWER_UP_DOWN           = 0x04,
    AD5676_COM_LDAC_MASK_REG_CONFIGURE = 0x05,
    AD5676_COM_SOFT_RESET              = 0x06,
    AD5676_COM_SETUP_REG_CONFIGURE     = 0x07,
    AD5676_COM_DCEN_REG_CONFIGURE      = 0x08,
    AD5676_COM_READBACK_REG_CONFIGURE  = 0x09,
    AD5676_COM_ALL_INP_REG_UPDATE      = 0x0A,
    AD5676_COM_ALL_INP_DAC_REG_UPDATE  = 0x0B,
    AD5676_COM_DAISY_CHAIN             = 0x0F // Not supported right now
} ad5676_command_t;

class AD5676 {
  public:
    AD5676() : spi_host_(&SPI), spi_settings_(20000000, MSBFIRST, SPI_MODE1) {};
    void init(const int8_t t_cs_pin, const int8_t t_ldac_pin, const int8_t t_rst_pin, SPIClass* t_spi_host = &SPI);
    void transfer(const ad5676_command_t t_command, const int8_t t_chan_addr_id, const std::array<uint8_t, 2>& t_data);
    void setLevel(const int8_t t_chan_addr_id, const uint16_t t_new_dac_level);

  private:
    SPIClass* spi_host_;
    SPISettings spi_settings_;
    int8_t cs_pin_;
    int8_t ldac_pin_;
    int8_t rst_pin_;
};

#endif