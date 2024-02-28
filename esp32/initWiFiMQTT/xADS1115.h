#include <ADS1X15.h>

#ifndef xADS1115_H
#define xADS1115_H

class xADS1115{
  public:
    xADS1115(int intruptPin, int gain, int dataRate);
    void beginADS();
    int16_t handleConversion();
    static void makeADSReady();
  private:
    ADS1115 ADS;
    int ADS_INTRUPT_PIN;
    int gain;
    int dataRate;
    static bool adsReady;
    static int count;
};

#endif