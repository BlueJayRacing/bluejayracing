#include "../sensors/ADS1115/adc.hpp"
#include "../ui/lcd2004/lcd.hpp"
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using std::remove;
using std::rename;
using std::string;
using std::to_string;
using std::vector;

char BUS_NAME = '2';

int main(/*int argc, char* argv[]*/)
{

    int fd     = open("/dev/i2c-6", O_RDWR);
    int fd_lcd = open("/dev/i2c-4", O_RDWR);

    LCD l = LCD(fd_lcd);

    for (int i = 0; i < 100000000; i++) {
        string s = to_string(a.read()[0] / 17500);
        // print
        cout << s << endl;
        l.write(0, 0, vector<char>(s.begin(), s.end()));
        usleep(1000.0 * (1.0 / 860.0));
        // usleep(200000);
    }

    // temp.close();

    close(fd);
    return 0;
    // 17000
}
