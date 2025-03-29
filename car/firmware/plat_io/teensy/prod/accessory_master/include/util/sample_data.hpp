#pragma once

#include <cstdint>
#include <string>
#include "teensy_mapping.hpp"

namespace baja {
namespace data {

/**
 * @brief Channel Sample Structure
 * 
 * Contains a single data sample with timestamp, internal channel ID, and value.
 * This is the basic unit of data flowing through the system.
 */
struct ChannelSample {
    uint64_t timestamp;                // Microsecond timestamp
    uint8_t internalChannelId;         // Internal channel ID (0-29)
    uint32_t rawValue;                 // Sensor value (24-bit for ADC, other values for different sensors)
    uint32_t recordedTimeMs;           // Millisecond timestamp when the sample was processed (optional)
    
    // Default constructor
    ChannelSample() : 
        timestamp(0), 
        internalChannelId(0), 
        rawValue(0),
        recordedTimeMs(0) {}
    
    // Constructor with main parameters
    ChannelSample(uint64_t ts, uint8_t chId, uint32_t val) : 
        timestamp(ts), 
        internalChannelId(chId), 
        rawValue(val),
        recordedTimeMs(0) {}
    
    // Constructor with all parameters
    ChannelSample(uint64_t ts, uint8_t chId, uint32_t val, uint32_t recTime) : 
        timestamp(ts), 
        internalChannelId(chId), 
        rawValue(val),
        recordedTimeMs(recTime) {}
        
    /**
     * @brief Convert sample to CSV format with minimal fields
     * 
     * @return CSV string representation of the sample (timestamp,channelID,value)
     */
    std::string toCSV() const {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%llu,%u,%lu", 
                 timestamp, internalChannelId, rawValue);
        return std::string(buffer);
    }
    
    // /**
    //  * @brief Convert sample to CSV format with all fields
    //  * 
    //  * @param includeChannelName Whether to include the channel name in CSV
    //  * @return Full CSV string representation of the sample
    //  */
    // std::string toFullCSV(bool includeChannelName = true) const {
    //     char buffer[128];
        
    //     if (includeChannelName) {
    //         std::string channelName = baja::util::getChannelName(internalChannelId);
    //         snprintf(buffer, sizeof(buffer), "%llu,%u,%u,\"%s\",%lu", 
    //                  timestamp, recordedTimeMs, internalChannelId, 
    //                  channelName.c_str(), rawValue);
    //     } else {
    //         snprintf(buffer, sizeof(buffer), "%llu,%u,%u,%lu", 
    //                  timestamp, recordedTimeMs, internalChannelId, rawValue);
    //     }
        
    //     return std::string(buffer);
    // }
};

} // namespace data
} // namespace baja