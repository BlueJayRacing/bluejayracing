
#include "magnetometer.hpp"

Magnetometer::Magnetometer() 
    : sensor(Wire, TLx493D_IIC_ADDR_A0_e) {}

bool Magnetometer::begin() {
    Wire.begin();
    return sensor.begin();
}

bool Magnetometer::readRawMag(int16_t &x, int16_t &y, int16_t &z) {
    return sensor.getRawMagneticField(&x, &y, &z);
}

int16_t Magnetometer::readMagX(int16_t &x) {
    return x;
}

int16_t Magnetometer::readMagY(int16_t &y) {
    return y;
}

int16_t Magnetometer::readMagZ(int16_t &z) {
    return z;
}
