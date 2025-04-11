#pragma once

#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include <Arduino.h>

namespace baja {
namespace digital {

// Constants for the digital input pins
constexpr uint8_t D1_PIN = 27;
constexpr uint8_t D2_PIN = 26;
constexpr uint8_t D3_PIN = 39;
constexpr uint8_t D4_PIN = 38;
constexpr uint8_t D5_PIN = 41;
constexpr uint8_t D6_PIN = 40;

// Number of digital channels
constexpr uint8_t DIGITAL_CHANNEL_COUNT = 6;

// Internal channel IDs for digital channels (matching the mapping in teensy_mapping.hpp)
constexpr uint8_t DIGITAL_CHANNEL_ID_START = 16; // As per the mapping

namespace functions {

/**
 * @brief Initialize the digital input module
 * 
 * @param mainBuffer Reference to the main ring buffer for SD storage
 * @return true if initialization was successful
 */
bool initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& mainBuffer);

/**
 * @brief Start digital input monitoring
 * 
 * @return true if successful
 */
bool start();

/**
 * @brief Stop digital input monitoring
 * 
 * @return true if successful
 */
bool stop();

/**
 * @brief Check if digital input monitoring is running
 * 
 * @return true if running
 */
bool isRunning();

/**
 * @brief Process digital inputs - main function called in master loop
 * 
 * @return true if samples were processed
 */
bool process();

/**
 * @brief Get timing statistics for digital input processing
 * 
 * @param avgTime Average processing time in microseconds
 * @param minTime Minimum processing time in microseconds
 * @param maxTime Maximum processing time in microseconds
 * @param sampleCount Total samples processed
 */
void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint64_t& sampleCount);

/**
 * @brief Reset timing statistics
 */
void resetTimingStats();

/**
 * @brief Get the total sample count
 * 
 * @return Number of samples processed
 */
uint64_t getSampleCount();

} // namespace functions

} // namespace digital
} // namespace baja