#include <ADS1X15.h>

#ifndef XADS1115_H
#define XADS1115_H

class xADS1115{
  public:
    xADS1115(int ads_intrupt_pin, int gain, int data_rate);
    void beginADS();
    int16_t handleConversion();
    static void makeADSReady();
  private:
    ADS1115 ADS;
    int ads_intrupt_pin;
    int gain;
    int data_rate;
    static bool ads_ready;
};

#endif