
#include "magnetometer.hpp"

Magnetometer::Magnetometer() : sensor(Wire, TLx493D_IIC_ADDR_A0_e) {}

bool Magnetometer::begin() {
    Wire.begin();
    return sensor.begin();
}

bool Magnetometer::readRawMag(double &x, double &y, double &z) {
    return sensor.getRawMagneticField(&x, &y, &z);
}

double Magnetometer::readMagX(double &x) {
    return x;
}

double Magnetometer::readMagY(double &y) {
    return y;
}

double Magnetometer::readMagZ(double &z) {
    return z;
}
