#pragma once
#ifndef _AD5626_H_
#define _AD5626_H_

#include <Arduino.h>
#include <SPI.h>

class AD5626 {
  public:
    AD5626(void) : m_spi_host(&SPI){};
    void init(int8_t m_cs_pin, int8_t t_ldac_pin, int8_t t_clr_pin, SPIClass* t_spi_host = &SPI);
    void setLevel(uint16_t t_new_level);
    void resetLevel(void);

  private:
    SPIClass* m_spi_host;
    int8_t m_cs_pin;
    int8_t m_ldac_pin;
    int8_t m_clr_pin;
};

#endif