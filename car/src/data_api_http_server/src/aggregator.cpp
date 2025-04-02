#include "aggregator.hpp"
#include "channel_mapping.hpp"  // For semantic channel names.
#include <algorithm>
#include <iostream>

namespace baja {
namespace aggregator {

Aggregator::Aggregator() {
    // Default config is already set.
}

void Aggregator::updateConfig(const AggregationConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

AggregationConfig Aggregator::getConfig() {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void Aggregator::insertSample(const std::string & sourceId, const baja::data::ChannelSample& sample, uint8_t channelIndex) {
    std::lock_guard<std::mutex> lock(mutex_);
    ChannelKey key { sourceId, sample.channelId };
    auto& deque = channelSamples_[key];
    if (deque.empty() || sample.timestamp >= deque.back().timestamp) {
        deque.push_back(sample);
    } else {
        auto it = std::upper_bound(deque.begin(), deque.end(), sample.timestamp,
                                   [](uint64_t ts, const baja::data::ChannelSample& s) { return ts < s.timestamp; });
        deque.insert(it, sample);
    }
    if (mappingInfo_.find(key) == mappingInfo_.end()) {
        MappingInfo info;
        info.sourceId = sourceId;
        info.channelId = sample.channelId;
        info.channelIndex = channelIndex;
        info.channelName = baja::util::getChannelNameString(channelIndex);
        mappingInfo_[key] = info;
    }
    uint64_t latestTimestamp = deque.back().timestamp;
    removeExpiredSamples(deque, latestTimestamp);
}

void Aggregator::removeExpiredSamples(std::deque<baja::data::ChannelSample>& samples, uint64_t latestTimestamp) {
    uint64_t cutoff = (latestTimestamp > config_.aggregationWindowMicroseconds)
                        ? latestTimestamp - config_.aggregationWindowMicroseconds
                        : 0;
    while (!samples.empty() && samples.front().timestamp < cutoff) {
        samples.pop_front();
    }
}

std::vector<baja::data::ChannelSample> Aggregator::getAggregatedSamples(const std::string & sourceId) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<baja::data::ChannelSample> result;
    for (const auto& kv : channelSamples_) {
        if (kv.first.sourceId == sourceId) {
            const auto& deque = kv.second;
            size_t factor = config_.downSamplingFactor;
            for (size_t i = 0; i < deque.size(); i += factor) {
                result.push_back(deque[i]);
            }
        }
    }
    return result;
}

std::vector<baja::data::DataChunk> Aggregator::getAggregatedDataAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, baja::data::DataChunk> chunks;
    for (const auto& kv : channelSamples_) {
        std::string src = kv.first.sourceId;
        auto& deque = kv.second;
        size_t factor = config_.downSamplingFactor;
        for (size_t i = 0; i < deque.size(); i += factor) {
            chunks[src].sourceId = src;
            chunks[src].samples.push_back(deque[i]);
        }
    }
    std::vector<baja::data::DataChunk> result;
    for (auto& kv : chunks) {
        result.push_back(kv.second);
    }
    return result;
}

std::vector<Aggregator::MappingInfo> Aggregator::getMappingInfo() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<MappingInfo> mappings;
    for (const auto& kv : mappingInfo_) {
        mappings.push_back(kv.second);
    }
    return mappings;
}

} // namespace aggregator
} // namespace baja
