#pragma once

#include "channel_sample.hpp"
#include <vector>
#include <sstream>
#include <nlohmann/json.hpp>

namespace baja {
namespace data {

struct DataChunk {
    std::string sourceId;  // Updated to string.
    std::vector<ChannelSample> samples;

    DataChunk() : sourceId("") {}
    DataChunk(const std::string & src) : sourceId(src) {}

    // Iterate over samples and call each sample's toCSV().
    std::string toCSV() const {
        std::stringstream ss;
        for (const auto &sample : samples) {
            ss << sample.toCSV() << "\n";
        }
        return ss.str();
    }

    nlohmann::json toJson() const {
        nlohmann::json j;
        j["source_id"] = sourceId;
        j["aggregation_window"] = 10;      // seconds (default)
        j["down_sampling"] = 10;           // 1/10 (default)
        j["samples"] = nlohmann::json::array();
        for (const auto &sample : samples) {
            nlohmann::json s;
            s["timestamp"] = sample.timestamp;
            s["channel_index"] = sample.channelIndex;
            s["data_value"] = sample.dataValue;  // Updated member name.
            s["channel_id"] = sample.channelId;
            j["samples"].push_back(s);
        }
        return j;
    }
};

} // namespace data
} // namespace baja
