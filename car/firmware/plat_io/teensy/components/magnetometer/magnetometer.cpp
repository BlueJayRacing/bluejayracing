#include "magnetometer.hpp"

Magnetometer::Magnetometer() {}

bool Magnetometer::begin(TwoWire& wire, uint8_t addr) {
    wire.begin();
    if(!sensor.begin_I2C(addr, &wire)) {
        return false;
    }

    sensor.setGain(MLX90393_GAIN_1X);
    sensor.setOversampling(MLX90393_OSR_0);
    sensor.setFilter(MLX90393_FILTER_2);

    return true;
}

void Magnetometer::readRawMag(float &x, float &y, float &z) {
    sensor.readData(&x, &y, &z);
}