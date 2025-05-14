#include "aggregation_server.hpp"
#include <pistache/http.h>
#include <pistache/endpoint.h>
#include <iostream>
#include <chrono>
#include <thread>

namespace baja {
namespace server {

AggregationServer::AggregationServer(const rclcpp::NodeOptions& options)
    : Node("aggregation_server", options), httpServer_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Starting AggregationServer node...");
    aggregator_ = std::make_shared<baja::aggregator::Aggregator>();

    // Subscribe to the unified data chunk topic.
    subscription_ = this->create_subscription<baja_msgs::msg::DataChunk>(
        "unified_data_chunk", 100,
        std::bind(&AggregationServer::dataChunkCallback, this, std::placeholders::_1)
    );

    // Start the HTTP server.
    initHttpServer();
}

AggregationServer::~AggregationServer() {
    if (httpServer_) {
        RCLCPP_INFO(this->get_logger(), "Shutting down HTTP server...");
        httpServer_->shutdown();
        delete httpServer_;
    }
    if (httpServerThread_.joinable()) {
        httpServerThread_.join();
    }
}

void AggregationServer::dataChunkCallback(const baja_msgs::msg::DataChunk::SharedPtr msg) {
    // source_id is now a string (e.g., "teensy1")
    std::string sourceId = msg->source_id;

    // Iterate through each unified sample.
    for (const auto & sampleMsg : msg->samples) {
        // Use the device's recorded_time as the sample timestamp.
        uint64_t timestamp = sampleMsg.recorded_time;
        // Use internal_channel_id as the physical channel index.
        uint8_t channelIndex = sampleMsg.internal_channel_id;
        // Use data_value as the sample value.
        float dataValue = sampleMsg.data_value;
        // Use ros_channel_id as the unique channel identifier.
        uint32_t channelId = sampleMsg.ros_channel_id;
        
        // Create an internal ChannelSample.
        baja::data::ChannelSample sample(timestamp, channelIndex, dataValue, channelId);
        aggregator_->insertSample(sourceId, sample, channelIndex);
    }
    // RCLCPP_INFO(this->get_logger(), "Received unified DataChunk from source '%s' with %zu samples.",
    //             sourceId.c_str(), msg->samples.size());
}

void AggregationServer::initHttpServer() {
    Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(9365));
    auto opts = Pistache::Http::Endpoint::options().threads(2).flags(Pistache::Tcp::Options::ReuseAddr);
    httpServer_ = new Pistache::Http::Endpoint(addr);
    httpServer_->init(opts);
    auto handler = std::make_shared<AggregationHttpHandler>(aggregator_);
    httpServer_->setHandler(handler);
    httpServerThread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "HTTP server listening on port 9365");
        httpServer_->serve();
    });
}

} // namespace server
} // namespace baja
