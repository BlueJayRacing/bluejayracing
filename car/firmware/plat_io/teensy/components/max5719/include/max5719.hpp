#pragma once
#ifndef _MAX5719_HPP_
#define _MAX5719_HPP_

#include <Arduino.h>
#include <SPI.h>

class MAX5719 {
  public:
    MAX5719() : spi_host_(&SPI), spi_settings_(10000000, MSBFIRST, SPI_MODE0) {};
    void init(int8_t t_cs_pin, int8_t t_ldac_pin, SPIClass* t_spi_host = &SPI);
    void setLevel(uint32_t t_new_dac_level);

  private:
    SPIClass* spi_host_;
    SPISettings spi_settings_;
    int8_t cs_pin_;
    int8_t ldac_pin_;
};

#endif