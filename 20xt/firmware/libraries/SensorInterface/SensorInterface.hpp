#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include <vector>
#include <string>
#include <cstdint>

class SensorInterface {
    public:
    virtual uint16_t read(void) = 0;

    struct Channel {
        std::string channel_name;
        std::string unit_of_measurement;
        std::string scale_factor_decimal;
        std::string time_unit;
        std::string num_bits_per_sample;
    };
};

#endif