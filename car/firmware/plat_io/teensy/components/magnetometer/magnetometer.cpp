#include "magnetometer.hpp"

Magnetometer::Magnetometer() {}

bool Magnetometer::begin(TwoWire& wire, uint8_t addr) {
    wire.begin();
    if(!sensor.begin_I2C(addr, &wire)) {
        return false;
    }

    sensor.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    sensor.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    return true;
}

void Magnetometer::readRawMag(int16_t &x, int16_t &y, int16_t &z) {
    sensor.read();

    x = sensor.x;
    y = sensor.y;
    z = sensor.z;
}