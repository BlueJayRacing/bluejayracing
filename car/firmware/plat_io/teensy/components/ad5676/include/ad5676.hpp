#ifndef _AD5676_HPP_
#define _AD5676_HPP_

#include <Arduino.h>
#include <SPI.h>

class AD5676 {
  public:
    AD5676() {};
    void init();
  private:
    SPIClass* spi_host_;
};

#endif