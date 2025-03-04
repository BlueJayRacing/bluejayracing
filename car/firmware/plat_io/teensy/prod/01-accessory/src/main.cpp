#include <Arduino.h>

#include "ad717x.hpp"
#include <test.hpp>
#include <wsg_cal_com.pb.h>
#include <pb_common.h>

void setup()
{
    Test test;
    test.readAnalogChannels();
}

void loop()
{
}