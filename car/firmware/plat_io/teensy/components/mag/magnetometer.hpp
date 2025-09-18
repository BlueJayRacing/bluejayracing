#ifndef _MAGNETOMETER_HPP_
#define _MAGNETOMETER_HPP_

#include <Arduino.h>
#include <Wire.h>
#include "TLx493D_inc.hpp"


class Magnetometer {
public:
    Magnetometer();
    bool begin();
    bool readRawMag(double &x, double &y, double &z);
    double readMagX(double &x);
    double readMagY(double &y);
    double readMagZ(double &z);

private:
    ifx::tlx493d::TLx493D_A1B6 sensor;
};

#endif