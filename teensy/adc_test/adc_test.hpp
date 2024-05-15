#include <TeensyADS1120.h>

#ifndef xADS1120_H
#define xADS1120_H

class xADS1120{
    public:
        void begin(int clk_pin, int miso_pin, int mosi_pin, int cs_pin, int drdy_pin, int spi_num);
        uint16_t readADC();
        int readADCSingle();
        void setMultiplexer(int mux);
        uint8_t readRegister(uint8_t address);
        void reset();
    private:
        TeensyADS1120* adc;
};

#endif