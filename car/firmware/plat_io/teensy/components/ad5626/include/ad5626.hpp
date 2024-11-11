#pragma once
#ifndef _AD5626_HPP_
#define _AD5626_HPP_

#include <Arduino.h>
#include <SPI.h>

class AD5626 {
  public:
    AD5626(void) : spi_host_(&SPI), spi_settings_(1000000, MSBFIRST, SPI_MODE3) {};
    void init(int8_t t_cs_pin, int8_t t_ldac_pin, int8_t t_clr_pin, SPIClass* t_spi_host = &SPI);
    void setLevel(uint16_t t_new_level);
    void resetLevel(void);

  private:
    SPIClass* spi_host_;
    SPISettings spi_settings_;
    int8_t cs_pin_;
    int8_t ldac_pin_;
    int8_t clr_pin_;
};

#endif