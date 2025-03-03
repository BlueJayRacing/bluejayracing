#include <Arduino.h>

#include "ad717x.hpp"
#include <test.hpp>

void setup()
{
    Test test;
    test.readAnalogChannels();
}

void loop()
{
}