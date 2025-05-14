// include/ntp/time_functions.hpp
#pragma once

#include <Arduino.h>
#include <TimeLib.h>
#include <AsyncUDP_Teensy41.hpp>

namespace baja {
namespace time {
namespace functions {

/**
 * @brief Initialize the time functions module (NTP UDP client).
 *
 * This sets up the UDP callback for NTP responses and any other initialization.
 *
 * @return true if initialized successfully.
 */
bool initialize();

/**
 * @brief Non-blocking update function.
 *
 * Call this from the master loop. It checks if UPDATE_INTERVAL_MS has elapsed
 * and sends an NTP request (via non-blocking UDP calls). The response handling
 * implements hierarchical updates: the fast RTC is updated if the difference is
 * >500 µs, while the battery-backed SRTC is updated only if the difference exceeds
 * 50 ms. Otherwise, the respective offsets are tracked for correct ground-truth.
 */
void update();

/**
 * @brief Get the last successful NTP Unix epoch time (seconds, UTC).
 *
 * @return time_t Last NTP time.
 */
time_t getLastNTPTime();

} // namespace functions
} // namespace time
} // namespace baja
