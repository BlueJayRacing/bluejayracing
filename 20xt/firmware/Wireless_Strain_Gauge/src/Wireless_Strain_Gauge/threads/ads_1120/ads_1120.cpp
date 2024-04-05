#include "ads_1120.h"

void ads_1120::begin(int cs_pin, int drdy_pin) {
  adc = new ADS1120();
  adc->begin(8, 9, 10, cs_pin, drdy_pin);
  attachInterrupt(digitalPinToInterrupt(drdy_pin), ads_1120::interrupt, FALLING);
  adc->setGain(1);               //Set gain 1.  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
  adc->setOpMode(0x02);          // Set Turbo Mode
  adc->setDataRate(0x06);        // Set Data rate 110.
  adc->setConversionMode(0x01);  // 1 = Continous Mode
  adc->setMultiplexer(0x04);     // Set AIN1  & GND
}

/*
 * This function checks the drdy_flag to see if a value has been converted on the ADC. If so, it
 * will return said value, and if not, it will return null.
 */
uint16_t ads_1120::read_adc() {
  if (drdy_flag) {
    drdy_flag = false;
    uint16_t val = adc->readADC();
    return val;
  }

  return NULL;
}

/*
 *This is the function attached to the interrupt on the drdy_pin, which turns on the drdy_flag.
 */

void ads_1120::interrupt() {
  drdy_flag = true;
}

bool ads_1120::drdy_flag = false;