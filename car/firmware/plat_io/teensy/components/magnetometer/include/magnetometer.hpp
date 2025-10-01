#ifndef _MAGNETOMETER_HPP_
#define _MAGNETOMETER_HPP_

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MLX90393.h"


class Magnetometer { 
public: 
    Magnetometer(); 
    bool begin(TwoWire& wire, uint8_t addr); 
    void readRawMag(float &x, float &y, float &z); 

private:
    Adafruit_MLX90393 sensor;
};

#endif