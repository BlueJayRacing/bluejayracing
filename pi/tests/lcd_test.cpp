#include "../ui/lcd2004/lcd.hpp"
#include <iostream>
#include <vector>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

string outvec(vector<double> v)
{
    string out = "";
    for (int i = 0; i < (int)v.size() - 1; i++) {
        out += to_string(v[i]) + ",";
    }

    if (v.size() > 0)
        out += to_string(v[v.size() - 1]);
    return out;
}

int main(/*int argc, char* argv[]*/)
{

    int fd = open("/dev/i2c-4", O_RDWR);
    if (fd < 0) {
        cerr << "Failed to open i2c bus (%c) ";
    }

    LCD l = LCD(fd);

    string s1 = "Hyun";
    string s2 = "second string";

    // row, column
    l.write(0, 0, vector<char>(s1.begin(), s1.end()));
    l.write(1, 0, vector<char>(s2.begin(), s2.end()));

    close(fd);
    return 0;
}
