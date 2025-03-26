#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <ad7175_8_regs.hpp>
#include <ad717x.hpp>

class Test {
public:
    void readAnalogChannels(void);
    void readDigitalChannels(void);
    void continuousChannelReadStats(void);
private:
    AD717X ad7175;
};

#endif