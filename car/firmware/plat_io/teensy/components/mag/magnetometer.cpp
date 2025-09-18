
#include "magnetometer.hpp"

Magnetometer::Magnetometer(TwoWire &w, TLx493D_IICAddressType_t addr) 
  : sensor(w, addr) {}

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
