#include "../sensors/ISM330DHCX/accelerometer.hpp"
#include <iostream>
#include "../ui/lcd2004/lcd.hpp"
#include <vector>


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


int main(/*int argc, char* argv[]*/) {
  int fd = open("/dev/i2c-1", O_RDWR);

  if (fd < 0) {
    char err[200];
    sprintf(err, "Failed to open i2c bus (%c) ", BUS_NAME);
  }

  if (fd < 0) {
    cerr <<"Failed to open i2c bus (%c) ";
  }
  


  Accelerometer a = Accelerometer(fd, 0);
   LCD l = LCD(fd);

  vector<double> data = a.read();
  
  for (int i = 0; i < 100; i++) {
    vector<double> data = a.read();
    cout << data[0] << ", " << data[1] << ", " << data[2] << endl;
    
    string s1 = to_string(data[0]);
    string s2 = to_string(data[1]);
    string s3 = to_string(data[2]);






  // string s1 = "ben leherh is a jayz";
  // string s2 = "small littler"+string(1, 0b11000011)+ string(1, 0b00000001);;
  // string s3 = "bunny and";
  // string s4 = "he is demanding";
  
  // row, column
  l.write(0, 0, vector<char>(s1.begin(), s1.end()));
	l.write(0, 1, vector<char>(s2.begin(), s2.end()));
	l.write(0, 2, vector<char>(s3.begin(), s3.end()));

  }


  close(fd);






  return 0;
}