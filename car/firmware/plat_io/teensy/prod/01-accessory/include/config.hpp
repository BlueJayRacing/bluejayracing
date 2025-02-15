#pragma once
#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#include <string>
#include <ad717x.hpp>

typedef struct access_digital {
    uint8_t digital_pin;
} access_digital_t;

typedef enum access_analog_type {
    STRAIN,
    LIMPOT,
    PRESSURE,
} access_analog_type;

typedef struct access_analog {
    access_analog_type type;
    ad717x_analog_input pos;
    ad717x_analog_input neg;
} access_analog_t;

typedef struct access_magnetic {
} access_magnetic_t;

typedef enum access_channel_type {
    ANALOG,
    DIGITAL,
    MAGNETIC,
} access_channel_type_t;

typedef struct access_channel_config {
    std::string channel_name;
    access_channel_type_t index;
    union {
        access_analog analog;
        access_digital_t digital;
        access_magnetic_t magnetic;
    };
    uint8_t buf_size_kB;
} access_channel_config_t;

const std::vector<access_channel_config_t> channel_configs = 
{
    {"strain_left", ANALOG, .analog = {STRAIN, AIN0, REF_M}, 40},
    {"hal_left", DIGITAL, .digital = {1}, 40},
    {"hal_right", DIGITAL, .digital = {5}, 40}
};

#endif