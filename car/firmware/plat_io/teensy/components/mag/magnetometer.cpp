#include "magnetometer.hpp"
#include <Adafruit_TLV493D.h>

Adafruit_TLV493D mag = Adafruit_TLV493D();

void Magnetometer::begin() {
    mag.begin();
}

bool Magnetometer::readMagData(int16_t &x, int16_t &y, int16_t &z) {
    /*
    // Request 7 bytes from the sensor and return, if it didn't send enough (do not read, if there is nothing to read)
    if(Wire.requestFrom(tlv_addr, 7) < 7) return false;
    // Read all registers to variables
    byte bx_high = Wire.read();
    byte by_high = Wire.read();
    byte bz_high = Wire.read();
    byte temp_high = Wire.read();
    byte bxy_low = Wire.read();
    byte bz_low = Wire.read();
    byte temp_low = Wire.read();

    unsigned int bx_value = (bx_high << 4) | ((bxy_low & 0xF0)>>4);
    unsigned int by_value = (by_high << 4) | (bxy_low & 0x0F);
    unsigned int bz_value = (bz_high << 4) | (bz_low & 0x0F);
    unsigned int temp_value = (temp_high >> 4) | temp_low;

    x = (int16_t)(bx_value << 4) / 16 ;
    y = (int16_t)(by_value << 4) / 16 ;
    z = (int16_t)(bz_value << 4) / 16 ;

    return true;
    */

    mag.read();  // updates internal sensor data

    //data output in Teslas
    x = mag.xData();
    y = mag.yData();
    z = mag.zData();

    return true;
}