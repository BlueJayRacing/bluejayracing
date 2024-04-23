#include "../ui/lcd2004/lcd.hpp"
#include "../sensors/ISM330DHCX/accelerometer.hpp"
#include <iostream>
#include <vector>
#include <unistd.h>

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::endl;
using std::to_string;

// TODO: Added definition for BUS_NAME as place holder
int BUS_NAME = 0x0;

string outvec(vector<double> v) {
  string out = "";
  for (int i = 0; i < (int)v.size()-1; i++) {
    out += to_string(v[i]) + ",";
  }

  if (v.size() > 0) out += to_string(v[v.size()-1]);
  return out;
}

uint8_t pj1[] = {
  0b10111,
  0b10001,
  0b10001,
  0b10111,
  0b10001,
  0b10001,
  0b10001,
  0b10111
};
uint8_t pj2[] = {
  0b11111,
  0b11111,
  0b11111,
  0b11001,
  0b00000,
  0b00000,
  0b10001,
  0b11110
};
uint8_t pj3[] = {
  0b11100,
  0b11110,
  0b11111,
  0b11111,
  0b00111,
  0b00001,
  0b11111,
  0b11111
};
uint8_t pj4[] = {
  0b01111,
  0b01011,
  0b01000,
  0b01000,
  0b01000,
  0b01000,
  0b01100,
  0b10110
};
uint8_t pj5[] = {
  0b01011,
  0b10001,
  0b10000,
  0b11100,
  0b00000,
  0b11110,
  0b00000,
  0b00000
};
uint8_t pj6[] = {
  0b10100,
  0b11000,
  0b00000,
  0b00011,
  0b00010,
  0b00011,
  0b00110,
  0b01010
};
uint8_t pj7[] = {
  0b00101,
  0b00100,
  0b00010,
  0b00010,
  0b00001,
  0b00000,
  0b00000,
  0b00000
};
uint8_t pj8[] = {
  0b11111,
  0b01110,
  0b01110,
  0b10101,
  0b00100,
  0b00000,
  0b00000,
  0b00000
};
uint8_t pj9[] = {
  0b00010,
  0b00100,
  0b00100,
  0b01000,
  0b10000,
  0b00000,
  0b00000,
  0b00000
};



int main(/*int argc, char* argv[]*/) {

  int fd = open("/dev/i2c-1", O_RDWR);

    
  if (fd < 0) {
    char err[200];
    sprintf(err, "Failed to open i2c bus (%c) ", BUS_NAME);
  }

  LCD l = LCD(fd);
  
  Accelerometer a = Accelerometer(fd, 0);


  vector<double> data = a.read();

  for (int i = 0; i < 100; i++) {
    vector<double> data = a.read();
    string s1 = std::to_string(data[0]);
    string s2 = std::to_string(data[1]);
    string s3 = std::to_string(data[2]);

    l.write(0, 0, vector<char>(s1.begin(), s2.end()));
    l.write(0, 1, vector<char>(s1.begin(), s2.end()));
    l.write(0, 2, vector<char>(s1.begin(), s2.end()));

    sleep(1);
  }


  close(fd);
  return 0;
}
