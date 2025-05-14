#pragma once

#include <cstdint>
#include <string>
#include <cstdio>
#include <inttypes.h>

namespace baja {
namespace data {

struct ChannelSample {
    uint64_t timestamp;      // Microsecond timestamp
    uint8_t channelIndex;    // Physical channel index
    float dataValue;         // Floating-point sensor value
    uint32_t channelId;      // Unique channel identifier

    ChannelSample() 
      : timestamp(0), channelIndex(0), dataValue(0.0f), channelId(0) {}

    ChannelSample(uint64_t ts, uint8_t ch, float val, uint32_t id)
      : timestamp(ts), channelIndex(ch), dataValue(val), channelId(id) {}

    std::string toCSV() const {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "%" PRIu64 ",%u,%.2f,%u", 
                 timestamp, channelIndex, dataValue, channelId);
        return std::string(buffer);
    }
};

} // namespace data
} // namespace baja
