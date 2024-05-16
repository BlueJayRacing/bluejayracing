#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

#include <vector>
#include <cstdint>

class SensorInterface {
    public:
    virtual void reset(void) = 0;
    virtual uint16_t read(void) = 0;
};

#endif