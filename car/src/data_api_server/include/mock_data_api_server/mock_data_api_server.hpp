#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pistache/endpoint.h>
#include <pistache/http.h>
#include <pistache/router.h>

#include <random>
#include <chrono>
#include <vector>
#include <map>
#include <mutex>
#include <deque>
#include <string>
#include <nlohmann/json.hpp>

namespace mock_data_api
{

// Time-Value data structure for sensor readings
struct TimeValue {
    uint64_t timestamp;  // nanoseconds since epoch
    double value;        // sensor value
};

// Channel types
enum class ChannelType {
    LINEAR_POTENTIOMETER,  // 2000Hz, voltage 0-5V
    HALL_EFFECT_SPEED,     // 2000Hz, RPM 0-10000
    BRAKE_PRESSURE,        // 2000Hz, PSI 0-2000
    NAVIGATION,            // 10000Hz, degrees 0-360
    STEERING_ENCODER,      // 2000Hz, degrees -180 to 180
    AXLE_TORQUE            // 2000Hz, Nm 0-500
};

// Channel configuration
struct ChannelConfig {
    ChannelType type;
    std::string name;
    double min_value;
    double max_value;
    double variance;      // Variance for the random walk
    double initial_value; // Starting value
};

// Data aggregator for a single channel
class ChannelAggregator {
public:
    ChannelAggregator(const ChannelConfig& config, size_t window_size);
    
    // Add a new sample at the current time
    void addSample(double value);
    
    // Get all samples within the window
    std::vector<TimeValue> getWindowData(size_t max_samples = 0) const;
    
    // Get the latest value
    TimeValue getLatestValue() const;
    
    // Get the channel configuration
    const ChannelConfig& getConfig() const;
    
private:
    ChannelConfig config_;
    std::deque<TimeValue> data_window_;
    mutable std::mutex mutex_;
    size_t window_size_;
};

// Main data generator and server handler
class MockDataGenerator {
public:
    MockDataGenerator(size_t window_size = 20000);  // Default 10s at 2000Hz = 20000 samples
    ~MockDataGenerator();
    
    // Initialize with default channels
    void initialize();
    
    // Start data generation (non-blocking)
    void startGeneration(double rate_hz = 10.0);  // 10Hz update rate
    
    // Stop data generation
    void stopGeneration();
    
    // Get data for all channels
    nlohmann::json getAllChannelsData(size_t max_samples_per_channel = 1000) const;
    
    // Get data for a specific channel
    nlohmann::json getChannelData(const std::string& channel_name, size_t max_samples = 1000) const;
    
private:
    // Generate a batch of data for all channels
    void generateDataBatch(size_t samples_per_batch);
    
    // Data generation thread function
    void generationThreadFunc();
    
    // Add a channel to monitor
    void addChannel(const ChannelConfig& config);
    
    // Random number generator
    std::mt19937 rng_;
    
    // Map of channel name to aggregator
    std::map<std::string, std::unique_ptr<ChannelAggregator>> channels_;
    
    // Window size for aggregation
    size_t window_size_;
    
    // Thread for data generation
    std::thread generation_thread_;
    bool should_stop_;
    
    // Rate control
    double generation_rate_hz_;
    size_t samples_per_generation_; // How many samples to generate per tick at the generation rate
};

// HTTP handler for the Pistache server
class MockDataAPIHandler : public Pistache::Http::Handler {
public:
    HTTP_PROTOTYPE(MockDataAPIHandler)
    
    explicit MockDataAPIHandler(std::shared_ptr<MockDataGenerator> generator);
    
    void onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response) override;
    
private:
    std::shared_ptr<MockDataGenerator> generator_;
};

// ROS2 node that hosts the Pistache server
class MockDataAPIServerNode : public rclcpp::Node {
public:
    MockDataAPIServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MockDataAPIServerNode();
    
private:
    void initServer();
    
    // Pistache server
    Pistache::Http::Endpoint* server_;
    
    // Data generator
    std::shared_ptr<MockDataGenerator> generator_;
};

} // namespace mock_data_api