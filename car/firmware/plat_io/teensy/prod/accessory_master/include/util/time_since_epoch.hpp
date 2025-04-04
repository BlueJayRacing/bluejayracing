#pragma once
#include <Arduino.h>
#include <stdint.h>

uint64_t getMicrosecondsSinceEpoch() {
  // Atomic read loop: read RTC registers until two consecutive reads are identical.
  uint32_t hi1 = SNVS_HPRTCMR;
  uint32_t lo1 = SNVS_HPRTCLR;
  while (true) {
    uint32_t hi2 = SNVS_HPRTCMR;
    uint32_t lo2 = SNVS_HPRTCLR;
    if (hi1 == hi2 && lo1 == lo2) {
      // The RTC registers are arranged as follows:
      // Seconds: (hi << 17) | (lo >> 15)
      // Fraction: lower 15 bits of lo (range: 0–32767) representing the sub-second ticks.
      uint32_t secs = (hi2 << 17) | (lo2 >> 15);
      uint32_t frac = lo2 & 0x7FFF;
      // Convert seconds to microseconds and add the fractional part.
      uint64_t us = ((uint64_t)secs * 1000000ULL) + (((uint64_t)frac * 1000000ULL) / 32768);
      return us;
    }
    hi1 = hi2;
    lo1 = lo2;
  }
}