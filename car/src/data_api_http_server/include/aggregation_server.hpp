#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <pistache/endpoint.h>
#include "aggregator.hpp"
#include "http_handler.hpp"
#include "baja_msgs/msg/data_chunk.hpp"  // ROS message definition for DataChunk

namespace baja {
namespace server {

class AggregationServer : public rclcpp::Node {
public:
    AggregationServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AggregationServer();

private:
    void dataChunkCallback(const baja_msgs::msg::DataChunk::SharedPtr msg);
    void initHttpServer();

    std::shared_ptr<baja::aggregator::Aggregator> aggregator_;
    rclcpp::Subscription<baja_msgs::msg::DataChunk>::SharedPtr subscription_;

    Pistache::Http::Endpoint* httpServer_;
    std::thread httpServerThread_;
};

} // namespace server
} // namespace baja
