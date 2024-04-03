#include "xADS1120.hpp"

void xADS1120::begin(int cs_pin, int drdy_pin) {
  adc = new ADS1120();
  adc->begin(8, 9, 10, cs_pin, drdy_pin);
  attachInterrupt(digitalPinToInterrupt(drdy_pin), xADS1120::interrupt, FALLING);
  adc->setGain(1);               //Set gain 1.  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
  adc->setOpMode(0x02);          // Set Turbo Mode
  adc->setDataRate(0x06);        // Set Data rate 110.
  adc->setConversionMode(0x01);  // 1=Continous Mode
  adc->setMultiplexer(0x09);     // Set AIN1  & GND
}

uint16_t xADS1120::readADC() {
  if (drdy_flag) {

    drdy_flag = false;
    uint16_t val = adc->readADC();
    return val;
  }

  return NULL;
}

void xADS1120::interrupt() {
  drdy_flag = true;
}

bool xADS1120::drdy_flag = false;