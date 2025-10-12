#ifndef _MAGNETOMETER_HPP_
#define _MAGNETOMETER_HPP_

#include <Arduino.h>
#include <Wire.h>
#include "Tlv493d.h"


class Magnetometer { 
public: 
    Magnetometer(); 
    void begin(TwoWire& wire, Tlv493d_Address_t); 
    void readRawMag(float &x, float &y, float &z); 
    float readMagX(); 
    float readMagY(); 
    float readMagZ();
    uint16_t getMeasurementDelay();

private:
    Tlv493d sensor;
};

#endif