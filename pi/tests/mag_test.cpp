#include "../sensors/LIS3MDL/magnetometer.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

using std::cout;
using std::endl;
using std::vector;

// TODO: Added definition for BUS_NAME as place holder
int BUS_NAME = 0x0;


int main(/*int argc, char* argv[]*/) {

  std::ofstream temp;
  temp.open("magnetometer_test.txt");
  temp << std::fixed << std::showpoint << std::setprecision(6);

  int fd = open("/dev/i2c-1", O_RDWR);
  if (fd < 0) {
    char err[200];
    sprintf(err, "Failed to open i2c bus (%c) ", BUS_NAME);
  }
  Magnetometer a = Magnetometer(fd, 0);

  vector<double> data = a.read();
  
  for (int i = 0; i < 100; i++) {
    vector<double> data = a.read();
    temp << data[0]  << ", " << data[1] << ", " << data[2] << endl;
    cout << data[0] << ", " << data[1] << ", " << data[2] << endl;
  }

  temp.close();

  close(fd);
  return 0;
}