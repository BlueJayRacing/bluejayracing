#pragma once
#ifndef _TEST_VERIFY_PSRAM_
#define _TEST_VERIFY_PSRAM_

#include <Arduino.h>

void test_check_psram();
bool check_fixed_pattern(uint32_t pattern);
bool check_lfsr_pattern(uint32_t seed);

#endif