#include "ads_1120.h"

int ads_1120::adc_number = 0;
bool ads_1120::drdy_flags[2] = {false};

ads_1120::ads_1120(void) : adc() {}

void ads_1120::begin(int cs_pin, int drdy_pin)
{
  adc.begin(8, 9, 10, cs_pin, drdy_pin);

  adc.setGain(1);              // Set gain 1.  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
  adc.setOpMode(0x02);         // Set Turbo Mode
  adc.setDataRate(0x06);       // Set Data rate 110. 2k SPS @ Turbo Mode
  adc.setConversionMode(0x01); // 1 = Continous Mode
  adc.setVoltageRef(1);        // Voltage Reference is External on REFP0 and REFN0 inputs
  adc.setMultiplexer(0x09);    // Recording Differential Between | AIN1 | AVSS |

  id = adc_number;
  drdy_flags[id] = false;

  switch (adc_number++) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(drdy_pin), ads_1120::interrupt0, FALLING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(drdy_pin), ads_1120::interrupt1, FALLING);
      break;
  }
}

/*
 * This function checks the drdy_flag to see if a value has been converted on the ADC. If so, it
 * will return said value, and if not, it will return null.
 */
uint16_t ads_1120::read_adc()
{
  if (drdy_flags[id])
  {
    drdy_flags[id] = false;
    uint16_t val = adc.readADC();
    return val;
  }

  return NULL;
}

/*
 *These are the functions attached to the interrupts on the drdy_pins, which turns on the drdy_flags.
 */
void ads_1120::interrupt0()
{
  drdy_flags[0] = true;
}

void ads_1120::interrupt1()
{
  drdy_flags[1] = true;
}