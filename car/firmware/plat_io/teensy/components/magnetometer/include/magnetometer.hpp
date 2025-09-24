#ifndef _MAGNETOMETER_HPP_
#define _MAGNETOMETER_HPP_

#include <Arduino.h>
#include <Wire.h>
#include "TLx493D_inc.hpp"


class Magnetometer { 
public: 
    Magnetometer(TwoWire wire); 
    bool begin(); 
    bool readRawMag(int16_t &x, int16_t &y, int16_t &z); 
    int16_t readMagX(int16_t &x); 
    int16_t readMagY(int16_t &y); 
    int16_t readMagZ(int16_t &z);

private:
    ifx::tlx493d::TLx493D_A1B6 sensor;
    TwoWire i2c_wire;
};

#endif