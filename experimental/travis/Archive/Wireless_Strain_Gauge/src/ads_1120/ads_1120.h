#ifndef ADS_1120_H
#define ADS_1120_H

#include <ADS1120.h>

/* This is a wrapper class for the TI ADS1120 Analog-to-Digital Converter.
 * It automatically configures the ADS1120 to be on continuous mode, with
 * a sampling rate of 2000 sps, with the user_inputted drdy_pin acting as
 * the interrupt pin on the IoT device.
 */

class ads_1120 {
  public:
    void begin(int cs_pin, int drdy_pin);
    uint16_t read_adc();

  private:
    ADS1120* adc;
    static void interrupt();
    static bool drdy_flag;
};

#endif