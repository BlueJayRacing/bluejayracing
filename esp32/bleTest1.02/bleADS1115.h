#include <ADS1X15.h>

#ifndef BLEADS1115_H
#define BLEADS1115_H

class bleADS1115{
  public:
    bleADS1115(int intruptPin, int gain, int dataRate);
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