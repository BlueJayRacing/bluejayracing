#pragma once

#include <cstdint>
#include "adc_channel_config.hpp"

namespace baja {
namespace data {

/**
 * @brief Channel Sample Structure
 * 
 * Contains a single ADC sample with timestamp, channel information, and raw value
 */
struct ChannelSample {
    uint64_t timestamp;      // Microsecond timestamp
    uint8_t channelIndex;    // Physical channel index
    uint32_t rawValue;       // Raw ADC value

    // Constructor for easy initialization
    ChannelSample(uint64_t ts = 0, uint8_t ch = 0, uint32_t val = 0)
        : timestamp(ts), channelIndex(ch), rawValue(val) {}
};

// Define the MQTT message size for optimal transmission
constexpr size_t MQTT_OPTIMAL_MESSAGE_SIZE = 1024; // 1KB chunks (adjust as needed)

// Define maximum possible downsampling ratio
constexpr uint8_t MAX_DOWNSAMPLE_RATIO = 100;

} // namespace data
} // namespace baja