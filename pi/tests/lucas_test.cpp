#include "../sensors/ISM330DHCX/accelerometer.hpp"
#include <iostream>
#include "../ui/lcd2004/lcd.hpp"
#include <vector>
#include <unistd.h>

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::cerr;
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
    Accelerometer a = Accelerometer(fd, 0);

    vector<double> data = a.read();
    

    
    LCD l = LCD(fd);
    
    l.createChar(0b00000000, pj1);
    l.createChar(0b00000001, pj2);
    l.createChar(0b00000010, pj3);
    l.createChar(0b00000011, pj4);
    l.createChar(0b00000100, pj5);
    l.createChar(0b00000101, pj6);
    l.createChar(0b00000110, pj7);
    l.createChar(0b00000111, pj8);


  
    //string s1 = string(1, 0b00000000)+string(1, 0b00000001)+string(1, 0b00000010)+string(1, 0b00000000)+string(1, 0b00000001)+string(1, 0b00000010)+string(1, 0b00000000)+string(1, 0b00000001)+string(1, 0b00000010)+ string(1, 0b00000000)+string(1, 0b00000001)+string(1, 0b00000010)+string(1, 0b00000000)+string(1, 0b00000001)+string(1, 0b00000010)+string(1, 0b00000000)+string(1, 0b00000001)+string(1, 0b00000010);
    //string s2 = string(1, 0b00000011)+string(1, 0b00000100)+string(1, 0b00000101)+string(1, 0b00000011)+string(1, 0b00000100)+string(1, 0b00000101)+string(1, 0b00000011)+string(1, 0b00000100)+string(1, 0b00000101)+string(1, 0b00000011)+string(1, 0b00000100)+string(1, 0b00000101)+string(1, 0b00000011)+string(1, 0b00000100)+string(1, 0b00000101)+string(1, 0b00000011)+string(1, 0b00000100)+string(1, 0b00000101);
    //string s3 = string(1, 0b00000110)+string(1, 0b00000111)+string(1, 0b11010010)+string(1, 0b00000110)+string(1, 0b00000111)+string(1, 0b00001000)+string(1, 0b00000110)+string(1, 0b00000111)+string(1, 0b00001000)+string(1, 0b00000110)+string(1, 0b00000111)+string(1, 0b00001000)+string(1, 0b00000110)+string(1, 0b00000111)+string(1, 0b00001000)+string(1, 0b00000110)+string(1, 0b00000111)+string(1, 0b00001000);
    //string s4 = "party 88888";

    
    char buffer1[30];
    char buffer2[30];
    char buffer3[30];

    sprintf(buffer1, "%f", data[0]);
    sprintf(buffer2, "%f", data[1]);
    sprintf(buffer3, "%f", data[2]);

    string s1 = buffer1;
    string s2 = buffer2;
    string s3 = buffer3;
    string s4 = ":D";
  
    int running = true;
    while (running) {

        vector<double> data = a.read();
        //cout << data[0] << ", " << data[1] << ", " << data[2] << endl;

        usleep(500000);

        sprintf(buffer1, "%f", data[0]);
        sprintf(buffer2, "%f", data[1]);
        sprintf(buffer3, "%f", data[2]);

        s1 = buffer1;
        s2 = buffer2;
        s3 = buffer3;
        s4 = ":D";


        // row, column
        l.write(0, 0, vector<char>(s1.begin(), s1.end()));
        l.write(0, 1, vector<char>(s2.begin(), s2.end()));
        l.write(0, 2, vector<char>(s3.begin(), s3.end()));
        l.write(0, 3, vector<char>(s4.begin(), s4.end()));

    }

  close(fd);
  return 0;
}



