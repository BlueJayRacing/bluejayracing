#include "magnetometer.hpp"

Magnetometer::Magnetometer(TwoWire& wire, TLx493D_IICAddressType_t addr) : sensor(wire, addr), i2c_wire(wire) {}

bool Magnetometer::begin() {
    return sensor.begin();
}

bool Magnetometer::readRawMag(int16_t &x, int16_t &y, int16_t &z) {

    return sensor.getRawMagneticField(&x, &y, &z);
}

// uint8_t Magnetometer::test() {
//     return sensor.setIICAddress(TLx493D_IIC_ADDR_A1_e);
// }

int16_t Magnetometer::readMagX(int16_t &x) {
    return x;
}

int16_t Magnetometer::readMagY(int16_t &y) {
    return y;
}

int16_t Magnetometer::readMagZ(int16_t &z) {
    return z;
}
