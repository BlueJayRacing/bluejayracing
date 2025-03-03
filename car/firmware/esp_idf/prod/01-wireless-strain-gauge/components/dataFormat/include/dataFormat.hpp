#pragma once
#ifndef _DATA_POINT_HPP_
#define _DATA_POINT_HPP_

#include <esp_system.h>
#include <array>

/**
 * This .hpp file defines the structs used to pass around
 * recorded data on the microcontroller. This file does not
 * describe the data format being passed from microcontroller
 * to Raspberry Pi.
 */

typedef struct data_point {
  float data_val;
  uint32_t time_offset_micro;
} data_point_t;

#define MAX_DATA_ARRAY_SIZE 2048
#define NUM_POINTS_IN_ARRAY (MAX_DATA_ARRAY_SIZE - sizeof(uint64_t)) / sizeof(data_point_t)

typedef struct data_array {
  uint64_t time_start_micro;
  std::array<data_point_t, NUM_POINTS_IN_ARRAY> array;
} data_array_t;

#endif