#include <ADS1120.h>

#ifndef xADS1120_H
#define xADS1120_H

class xADS1120 {
public:
  void begin(int cs_pin, int drdy_pin);
  uint16_t readADC();
private:
  ADS1120* adc;
  static void interrupt();
  static bool drdy_flag;
};

#endif