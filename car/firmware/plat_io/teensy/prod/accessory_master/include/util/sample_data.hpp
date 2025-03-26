#pragma once

#include <cstdint>
#include <string>

namespace baja {
namespace data {

/**
 * @brief Channel Sample Structure
 * 
 * Contains a single ADC sample with timestamp, channel information, and raw value.
 * This is the basic unit of data flowing through the system.
 */
struct ChannelSample {
    uint64_t timestamp;      // Microsecond timestamp
    uint8_t channelIndex;    // Physical channel index
    uint32_t rawValue;       // Raw ADC value (24-bit from AD7175-8)
    
    // Default constructor
    ChannelSample() : 
        timestamp(0), 
        channelIndex(0), 
        rawValue(0) {}
    
    // Constructor with all parameters
    ChannelSample(uint64_t ts, uint8_t ch, uint32_t val) : 
        timestamp(ts), 
        channelIndex(ch), 
        rawValue(val) {}
        
    /**
     * @brief Convert sample to CSV format
     * 
     * @return CSV string representation of the sample
     */
    std::string toCSV() const {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%llu,%u,%lu", 
                 timestamp, channelIndex, rawValue);
        return std::string(buffer);
    }
};

// Define the MQTT message size for optimal transmission
constexpr size_t MQTT_OPTIMAL_MESSAGE_SIZE = 1024; // 1KB chunks

// Define maximum possible downsampling ratio
constexpr uint8_t MAX_DOWNSAMPLE_RATIO = 1;

// Define file rotation interval in milliseconds (2 minutes)
constexpr uint32_t FILE_ROTATION_INTERVAL_MS =  30 * 1000;

} // namespace data
} // namespace baja

// External buffer declarations
extern uint8_t mqttMessageBuffer[];
extern uint8_t sdWriterBuffer[];
// extern baja::data::ChannelSample sdSampleBuffer[];
extern baja::data::ChannelSample ringBufferStorage[];