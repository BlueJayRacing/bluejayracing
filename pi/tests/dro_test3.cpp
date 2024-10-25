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
    if (fd < 0) {
        char err[200];
        sprintf(err, "Failed to open i2c bus (%c) ", BUS_NAME);
    }

    ADC a = ADC(fd, 3, true);

    LCD l = LCD(fd_lcd);

    for (int i = 0; i < 100000000; i++) {
        string s = to_string((a.read()[0] / 17000) * 10);
        // print
        cout << s << endl;
        l.write(0, 0, vector<char>(s.begin(), s.end()));
        usleep(1000.0 * (1.0 / 860.0));
        // usleep(200000);
    }

    temp.close();
    cout << a.read()[0] close(fd);
    return 0;
}
