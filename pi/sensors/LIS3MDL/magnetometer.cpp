/* IMPORTANT REGISTERS:

0b 0000 1111
WHO_AM_I: 0 0 1 1 1 1 0 1
    Device ID
i2cwrite9....,TEMP_EN | flag | flag
0b 0010 0000
#define TEMP_EN 0b10000000
#define XYZ_EN 0b00110000

CTRL_REG1 : TEMP_EN OM1 OM0 DO2 DO1 DO0 FAST_ODR ST
    DO[2:0] - Data output rate (Default: 100 Hz), range: (0.625, 80)
    FAST_ODR - Data output rates over 80 Hz
    ST - Enables self testing

0b 0010 0001 
CTRL_REG2: 0 FS1 FS0 0 REBOOT SOFT_RST 0 0
    [ZEROS NEED TO BE SET TO ZERO FOR THE DEVICE TO OPERATE]
    FS[1:0] -  Full scale selection ((+/-)4, 8, 12, 16)
  
Data output in 2's complement

    0b 0010 1000, 0b 0010 1001
    OUT_X_L, OUT_X_H;

    0b 0010 1010, 0b 0010 1011
    OUT_Y_L, OUT_Y_H;

    0b 0010 1100, 0b 0010 1101 
    OUT_Z_L, OUT_Z_H;

0b 0010 0010 
CTRL_REG3: 0 0 LP 0 0 SIM MD1 MD0
    LP - Low power mode config (Default: 0) 
    SIM - SPI serial interface mode selection (Default value: 0)
    MD[1:0] - Operating mode selection  (Default value: 11)
        00 - Continuous conversion

*/

#include "magnetometer.hpp"
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

Magnetometer::Magnetometer(): adr{LIS3MDL_ADDRESS}, fd{-1} {
  reset();
}

Magnetometer::Magnetometer(int fd): adr{LIS3MDL_ADDRESS}, fd{fd} {
  reset();
}

Magnetometer::Magnetometer(int fd, int location): adr{LIS3MDL_ADDRESS}, fd{fd} {
  adr = (location == 0 ? adr : LIS3MDL_ADDRESS2);
  reset();
}

Magnetometer::~Magnetometer() {
}

void Magnetometer::reset() {
  uint8_t* read = NULL;
  i2c_read(fd, adr, LIS3MDL_WHO_AM_I, read);
  if (*read != LIS3MDL_ID) {
    std::cerr << "WHO_AM_I does not match specification." << endl;
    exit(1);
  }
  
  i2c_write(fd, adr, LIS3MDL_REG_ONE, ODR_FAST | SELF_TEST);
  i2c_write(fd, adr, LIS3MDL_REG_TWO, FS);
  i2c_write(fd, adr, LIS3MDL_REG_THREE, OP_MODE_ON | LP_MODE);
  i2c_write(fd, adr, LIS3MDL_REG_FOUR, OMZ_LOW);
  i2c_write(fd, adr, LIS3MDL_REG_FIVE, FAST_READ_ON | BDU);

  return;
}

vector<double> Magnetometer::read() {
  vector<double> data = vector<double>();
  
  vector<uint8_t> buf = i2c_bulk_read(fd, adr, this->DATA_REGISTER, this->READ_LENGTH);

  data.push_back(static_cast<int16_t>(static_cast<uint16_t>(buf[1]) << 8 | static_cast<uint16_t>(buf[0]))*.000584);
	data.push_back(static_cast<int16_t>(static_cast<uint16_t>(buf[3]) << 8 | static_cast<uint16_t>(buf[2]))*.000584);
	data.push_back(static_cast<int16_t>(static_cast<uint16_t>(buf[5]) << 8 | static_cast<uint16_t>(buf[4]))*.000584);
  return data;
}