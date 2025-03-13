#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <ad7175_8_regs.hpp>
#include <ad717x.hpp>

class Test {
public:
    void readAnalogChannels(void);
    void readDigitalChannels(void);

private:
    static void D1InterruptFn(void);
    static void D2InterruptFn(void);
    static void D3InterruptFn(void);
    static void D4InterruptFn(void);
    static void D5InterruptFn(void);
    static void D6InterruptFn(void);

private:
    AD717X ad7175;
    
};

#endif