#include "../processing/fourier.hpp"
#include "../sensors/ADS1115/adc.hpp"
#include <deque>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <stdlib.h>
#include <string>

using std::cerr;
using std::cout;
using std::deque;
using std::endl;
using std::pair;
using std::remove;
using std::rename;
using std::to_string;
using std::vector;

int main(/*int argc, char* argv[]*/)
{

    std::ofstream temp;
    temp.open("temp.txt");
    temp << std::fixed << std::showpoint << std::setprecision(1);

    int fd = open("/dev/i2c-6", O_RDWR);
    if (fd < 0) {
        cerr << "Failed to open i2c bus 6" << endl;
    }

    ADC a           = ADC(fd, 1, false);
    deque<double> d = deque<double>();
    for (int i = 0; i < 2048; i++)
        d.push_front(0);

    temp << "Raw ADC output, FFT RPM, Amplitude" << endl;
    for (int i = 0; i < 8600; i++) {
        d.pop_front();
        vector<double> data = a.read();

        if (abs(data[0] < 2))
            data[0] = 0;
        d.push_back(data[0]);

        vector<cd> dft = vector<cd>(vector<cd>(d.begin(), d.end()));
        fft(dft, false);
        pair<double, double> max = pair<double, double>(0, 0);
        for (int j = 5; j < (int)dft.size() / 2; j++) {
            if (abs(dft[j]) > max.second && abs(dft[j]) > 1000) {
                max = pair<double, double>(j, abs(dft[j]));
            }
        }

        double rpm  = (max.first * 860 * 60) / (2.0 * 56.0 * 2048);
        double freq = max.first;
        temp << data[0] << ", " << rpm << ", " << max.second << endl;
        cout << data[0] << ", " << rpm << ", " << max.second << endl;
        usleep(1000);
        // a.swap_mux(0);
    }
    temp.close();

    close(fd);
    return 0;
}