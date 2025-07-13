#ifndef _MAGNETOMETER_HPP_
#define _MAGNETOMETER_HPP_

#include <Arduino.h>

class Magnetometer {
public:
    //Magnetometer(TwoWire &wire = Wire, uint8_t address = 0x5E);
    Magnetometer();
    void begin();
    bool readMagData(int16_t &x, int16_t &y, int16_t &z);
/*
private:
    TwoWire *_wire;
    uint8_t _address;

    void writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
*/
};

#endif
