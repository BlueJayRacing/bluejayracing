#ifndef _MAGNETOMETER_HPP_
#define _MAGNETOMETER_HPP_

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_LIS3MDL.h"


class Magnetometer { 
public: 
    Magnetometer(); 
    bool begin(TwoWire& wire, uint8_t addr); 
    void readRawMag(int16_t &x, int16_t &y, int16_t &z); 

private:
    Adafruit_LIS3MDL sensor;
};

#endif