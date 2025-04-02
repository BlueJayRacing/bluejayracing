#include "http_handler.hpp"
#include <iostream>
#include <sstream>

namespace baja {
namespace server {

void AggregationHttpHandler::onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response) {
    response.headers()
        .add<Pistache::Http::Header::AccessControlAllowOrigin>("*")
        .add<Pistache::Http::Header::ContentType>(MIME(Application, Json));

    std::string uri = request.resource();
    std::cout << "HTTP request: " << request.method() << " " << uri << std::endl;

    // Health check endpoint.
    if (uri == "/health") {
        nlohmann::json j; j["status"] = "OK";
        response.send(Pistache::Http::Code::Ok, j.dump(2));
        return;
    }
    // Config endpoint.
    if (uri == "/config" && request.method() == Pistache::Http::Method::Get) {
        auto config = aggregator_->getConfig();
        nlohmann::json j;
        j["aggregation_window_microseconds"] = config.aggregationWindowMicroseconds;
        j["max_messages"] = config.maxMessages;
        j["down_sampling_factor"] = config.downSamplingFactor;
        response.send(Pistache::Http::Code::Ok, j.dump(2));
        return;
    }
    // All data endpoint.
    if (uri == "/data/all" && request.method() == Pistache::Http::Method::Get) {
        auto dataChunks = aggregator_->getAggregatedDataAll();
        nlohmann::json j; j["data_chunks"] = nlohmann::json::array();
        for (const auto& chunk : dataChunks) {
            j["data_chunks"].push_back(chunk.toJson());
        }
        response.send(Pistache::Http::Code::Ok, j.dump(2));
        return;
    }
    // Data for a specific source endpoint.
    if (uri.find("/data/") == 0 && request.method() == Pistache::Http::Method::Get) {
        // Treat the portion after "/data/" as a string source id.
        std::string sourceId = uri.substr(std::string("/data/").size());
        auto samples = aggregator_->getAggregatedSamples(sourceId);
        nlohmann::json j;
        j["source_id"] = sourceId;
        j["samples"] = nlohmann::json::array();
        for (const auto& sample : samples) {
            nlohmann::json s;
            s["timestamp"] = sample.timestamp;
            s["channel_index"] = sample.channelIndex;
            s["data_value"] = sample.dataValue;
            s["channel_id"] = sample.channelId;
            j["samples"].push_back(s);
        }
        response.send(Pistache::Http::Code::Ok, j.dump(2));
        return;
    }
    // Mapping endpoint.
    if (uri == "/mapping" && request.method() == Pistache::Http::Method::Get) {
        auto mappings = aggregator_->getMappingInfo();
        nlohmann::json j; j["mappings"] = nlohmann::json::array();
        for (const auto& mapping : mappings) {
            nlohmann::json m;
            m["source_id"] = mapping.sourceId;
            m["channel_id"] = mapping.channelId;
            m["channel_index"] = mapping.channelIndex;
            m["channel_name"] = mapping.channelName;
            j["mappings"].push_back(m);
        }
        response.send(Pistache::Http::Code::Ok, j.dump(2));
        return;
    }
    // Unknown endpoint.
    nlohmann::json error; error["error"] = "Endpoint not found";
    response.send(Pistache::Http::Code::Not_Found, error.dump(2));
}

} // namespace server
} // namespace baja
