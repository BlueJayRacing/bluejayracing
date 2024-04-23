#include "../sensors/LIS3MDL/magnetometer.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <array>
#include <ios>
#include <time.h>


using std::cout;
using std::endl;
using std::vector;
using std::to_string;
using std::remove;
using std::rename;
using std::array;

using std::cout;
using std::endl;
using std::vector;

// TODO: Added definition for BUS_NAME as place holder
int BUS_NAME = 0x0;


int main(/*int argc, char* argv[]*/) {

  //make downloadable file
  std::ofstream perm;
  perm.open("rt_mag_pkg.txt");
  //set downloadable file synatx
  perm << std::fixed << std::showpoint << std::setprecision(6);

  //open i2c bus for sensor reading
  int fd = open("/dev/i2c-1", O_RDWR);
  if (fd < 0) {
    char err[200];
    sprintf(err, "Failed to open i2c bus (%c) ", BUS_NAME);
  }

  //create accelerometer object to read data
  Magnetometer a = Magnetometer(fd, 0);
  
  while(true) {

    //clock_t start, end;

    //start = clock();

    //create writing file
    std::ofstream write;
    write.open("rt_mag_wr.txt");
    //set writing file syntax
    write << std::fixed << std::showpoint << std::setprecision(6);

    //read 10000 data point and write them to the writing file
    for (int i = 0; i < 500; i++) {
      vector<double> data = a.read();
      write << data[0] << ", " << data[1] << ", " << data[2] << endl; 
    }

    //close the writing file
    write.close();
    //remove the old downloadable file
    remove("rt_mag_pkg.txt");
    //move the writing file to the downloadable file location
    rename("rt_mag_wr.txt", "rt_mag_pkg.txt");

    //end = clock();

    /*
    double time = ((double) (end - start)) / CLOCKS_PER_SEC;
    cout << time << endl;
    */
  }

  close(fd);
  return 0;
}