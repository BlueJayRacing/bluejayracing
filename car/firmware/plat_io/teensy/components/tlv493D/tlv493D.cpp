#include "tlv493D.hpp"

Magnetometer::Magnetometer() : sensor() {}

void Magnetometer::begin(TwoWire& wire, Tlv493d_Address_t addr) {
    sensor.begin(wire, addr, false);
    sensor.setAccessMode(sensor.MASTERCONTROLLEDMODE);
    sensor.disableTemp();
}

void Magnetometer::readRawMag(float &x, float &y, float &z) {
    sensor.updateData();
    x = sensor.getX();
    y = sensor.getY();
    z = sensor.getZ();
}

float Magnetometer::readMagX() {
    sensor.updateData();
    return sensor.getX();
}

float Magnetometer::readMagY() {
    sensor.updateData();
    return sensor.getY();
}

float Magnetometer::readMagZ() {
    sensor.updateData();
    return sensor.getZ();
}

uint16_t Magnetometer::getMeasurementDelay() {
    return sensor.getMeasurementDelay();
}


