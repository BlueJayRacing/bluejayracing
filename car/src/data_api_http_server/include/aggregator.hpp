#pragma once

#include "data/channel_sample.hpp"
#include "data/data_chunk.hpp"
#include <deque>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>

// Updated key: now uses a string for the source identifier.
struct ChannelKey {
    std::string sourceId;
    uint32_t channelId;

    bool operator==(const ChannelKey &other) const {
        return sourceId == other.sourceId && channelId == other.channelId;
    }
};

namespace std {
template <>
struct hash<ChannelKey> {
    std::size_t operator()(const ChannelKey& key) const {
        return std::hash<std::string>()(key.sourceId) ^ (std::hash<uint32_t>()(key.channelId) << 1);
    }
};
}

namespace baja {
namespace aggregator {

struct AggregationConfig {
    uint64_t aggregationWindowMicroseconds = 10000000;  // 10 seconds default.
    size_t maxMessages = 20000;
    size_t downSamplingFactor = 10;
};

class Aggregator {
public:
    Aggregator();

    void updateConfig(const AggregationConfig& config);
    AggregationConfig getConfig();

    // Use string for sourceId.
    void insertSample(const std::string & sourceId, const baja::data::ChannelSample& sample, uint8_t channelIndex);

    // Get aggregated samples for a given source (using a string identifier).
    std::vector<baja::data::ChannelSample> getAggregatedSamples(const std::string & sourceId);
    std::vector<baja::data::DataChunk> getAggregatedDataAll();

    // Mapping info structure.
    struct MappingInfo {
        std::string sourceId;
        uint32_t channelId;
        uint8_t channelIndex;
        std::string channelName;
    };
    std::vector<MappingInfo> getMappingInfo();

private:
    void removeExpiredSamples(std::deque<baja::data::ChannelSample>& samples, uint64_t latestTimestamp);

    std::unordered_map<ChannelKey, std::deque<baja::data::ChannelSample>> channelSamples_;
    std::unordered_map<ChannelKey, MappingInfo> mappingInfo_;
    AggregationConfig config_;
    std::mutex mutex_;
};

} // namespace aggregator
} // namespace baja
