#ifndef LIS3MDL_MAGNETOMETER
#define LIS3MDL_MAGNETOMETER

#include "../../communication/i2c_dev.hpp"
#include "../interfaces/sensor_interface.hpp"
#include "LIS3MDL.hpp"

class Magnetometer: public SensorInterface {
public:
  Magnetometer();
  Magnetometer(int fd);
  Magnetometer(int fd, int location);
  ~Magnetometer();
  
  void reset() override;
  std::vector<double> read() override;

private:
  static const int DATA_REGISTER = 0x28;
  static const int READ_LENGTH = 0x06;
  
  int adr;
  int fd;
};

#endif
