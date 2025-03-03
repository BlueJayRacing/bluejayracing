#include "mock_data_api_server/mock_data_api_server.hpp"
#include <chrono>
#include <thread>
#include <algorithm>
#include <sstream>

namespace mock_data_api
{

// ---- ChannelAggregator Implementation ----

ChannelAggregator::ChannelAggregator(const ChannelConfig& config, size_t window_size)
    : config_(config), window_size_(window_size)
{
    // Initialize with the starting value if needed
    if (config_.initial_value != 0) {
        auto now = std::chrono::system_clock::now();
        auto timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count());
        
        data_window_.push_back({timestamp, config_.initial_value});
    }
}

void ChannelAggregator::addSample(double value)
{
    auto now = std::chrono::system_clock::now();
    auto timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count());
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Add new sample
    data_window_.push_back({timestamp, value});
    
    // Remove old samples that exceed window size
    while (data_window_.size() > window_size_) {
        data_window_.pop_front();
    }
}

std::vector<TimeValue> ChannelAggregator::getWindowData(size_t max_samples) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<TimeValue> result;
    
    if (data_window_.empty()) {
        return result;
    }
    
    size_t count = max_samples > 0 ? std::min(max_samples, data_window_.size()) : data_window_.size();
    auto start_iter = data_window_.end() - count;
    
    result.insert(result.end(), start_iter, data_window_.end());
    return result;
}

TimeValue ChannelAggregator::getLatestValue() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (data_window_.empty()) {
        // Return a default value if no data
        return {0, config_.initial_value};
    }
    
    return data_window_.back();
}

const ChannelConfig& ChannelAggregator::getConfig() const
{
    return config_;
}

// ---- MockDataGenerator Implementation ----

MockDataGenerator::MockDataGenerator(size_t window_size)
    : window_size_(window_size), 
      should_stop_(false), 
      generation_rate_hz_(10.0),
      samples_per_generation_(200)  // 200 samples per tick at 10Hz = 2000Hz effective rate
{
    // Initialize random number generator with time-based seed
    std::random_device rd;
    rng_ = std::mt19937(rd());
}

MockDataGenerator::~MockDataGenerator()
{
    stopGeneration();
}

void MockDataGenerator::initialize()
{
    // Add default channels with reasonable configurations
    addChannel({ChannelType::LINEAR_POTENTIOMETER, "linpot_front_left", 0.0, 5.0, 0.005, 2.5});
    addChannel({ChannelType::LINEAR_POTENTIOMETER, "linpot_front_right", 0.0, 5.0, 0.005, 2.5});
    addChannel({ChannelType::LINEAR_POTENTIOMETER, "linpot_rear_left", 0.0, 5.0, 0.005, 2.5});
    addChannel({ChannelType::LINEAR_POTENTIOMETER, "linpot_rear_right", 0.0, 5.0, 0.005, 2.5});
    
    addChannel({ChannelType::HALL_EFFECT_SPEED, "wheel_speed_fl", 0.0, 10000.0, 10.0, 0.0});
    addChannel({ChannelType::HALL_EFFECT_SPEED, "wheel_speed_fr", 0.0, 10000.0, 10.0, 0.0});
    addChannel({ChannelType::HALL_EFFECT_SPEED, "wheel_speed_rl", 0.0, 10000.0, 10.0, 0.0});
    addChannel({ChannelType::HALL_EFFECT_SPEED, "wheel_speed_rr", 0.0, 10000.0, 10.0, 0.0});
    
    addChannel({ChannelType::BRAKE_PRESSURE, "brake_pressure_front", 0.0, 2000.0, 5.0, 0.0});
    addChannel({ChannelType::BRAKE_PRESSURE, "brake_pressure_rear", 0.0, 2000.0, 5.0, 0.0});
    
    addChannel({ChannelType::STEERING_ENCODER, "steering_angle", -180.0, 180.0, 0.5, 0.0});
    
    addChannel({ChannelType::AXLE_TORQUE, "axle_torque", 0.0, 500.0, 1.0, 0.0});
}

void MockDataGenerator::addChannel(const ChannelConfig& config)
{
    channels_[config.name] = std::make_unique<ChannelAggregator>(config, window_size_);
}

void MockDataGenerator::startGeneration(double rate_hz)
{
    if (generation_thread_.joinable()) {
        stopGeneration();
    }
    
    generation_rate_hz_ = rate_hz;
    samples_per_generation_ = static_cast<size_t>(2000.0 / rate_hz);  // To get 2000Hz effective rate
    
    should_stop_ = false;
    generation_thread_ = std::thread(&MockDataGenerator::generationThreadFunc, this);
}

void MockDataGenerator::stopGeneration()
{
    should_stop_ = true;
    if (generation_thread_.joinable()) {
        generation_thread_.join();
    }
}

void MockDataGenerator::generationThreadFunc()
{
    while (!should_stop_) {
        generateDataBatch(samples_per_generation_);
        
        // Sleep for the appropriate interval based on generation rate
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(1000.0 / generation_rate_hz_)));
    }
}

void MockDataGenerator::generateDataBatch(size_t samples_per_batch)
{
    for (auto& [name, channel] : channels_) {
        const auto& config = channel->getConfig();
        
        // Get the latest value as starting point
        double current_value = channel->getLatestValue().value;
        
        // Generate samples_per_batch new samples using a random walk
        std::normal_distribution<double> noise(0.0, config.variance);
        
        for (size_t i = 0; i < samples_per_batch; ++i) {
            // Add random noise, constrained by min/max
            current_value += noise(rng_);
            current_value = std::min(config.max_value, std::max(config.min_value, current_value));
            
            // Add the sample to the channel
            channel->addSample(current_value);
        }
    }
}

nlohmann::json MockDataGenerator::getAllChannelsData(size_t max_samples_per_channel) const
{
    nlohmann::json result;
    
    for (const auto& [name, channel] : channels_) {
        auto data = channel->getWindowData(max_samples_per_channel);
        
        // Convert data to JSON
        nlohmann::json channel_data = {
            {"name", name},
            {"type", static_cast<int>(channel->getConfig().type)},
            {"min_value", channel->getConfig().min_value},
            {"max_value", channel->getConfig().max_value},
            {"samples", nlohmann::json::array()}
        };
        
        for (const auto& sample : data) {
            channel_data["samples"].push_back({
                {"timestamp", sample.timestamp},
                {"value", sample.value}
            });
        }
        
        result["channels"].push_back(channel_data);
    }
    
    // Add metadata
    result["metadata"] = {
        {"timestamp", static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count())},
        {"channel_count", channels_.size()},
        {"max_samples_per_channel", max_samples_per_channel}
    };
    
    return result;
}

nlohmann::json MockDataGenerator::getChannelData(const std::string& channel_name, size_t max_samples) const
{
    nlohmann::json result;
    
    auto it = channels_.find(channel_name);
    if (it == channels_.end()) {
        result["error"] = "Channel not found: " + channel_name;
        return result;
    }
    
    const auto& channel = it->second;
    auto data = channel->getWindowData(max_samples);
    
    // Convert data to JSON
    result["name"] = channel_name;
    result["type"] = static_cast<int>(channel->getConfig().type);
    result["min_value"] = channel->getConfig().min_value;
    result["max_value"] = channel->getConfig().max_value;
    result["samples"] = nlohmann::json::array();
    
    for (const auto& sample : data) {
        result["samples"].push_back({
            {"timestamp", sample.timestamp},
            {"value", sample.value}
        });
    }
    
    // Add metadata
    result["metadata"] = {
        {"timestamp", static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count())},
        {"sample_count", data.size()},
        {"max_samples", max_samples}
    };
    
    return result;
}

// ---- MockDataAPIHandler Implementation ----

MockDataAPIHandler::MockDataAPIHandler(std::shared_ptr<MockDataGenerator> generator)
    : generator_(generator)
{
}

void MockDataAPIHandler::onRequest(const Pistache::Http::Request& request, Pistache::Http::ResponseWriter response)
{
    // Set CORS headers
    response.headers()
        .add<Pistache::Http::Header::AccessControlAllowOrigin>("*")
        .add<Pistache::Http::Header::AccessControlAllowMethods>("GET, POST, OPTIONS")
        .add<Pistache::Http::Header::AccessControlAllowHeaders>("Content-Type")
        .add<Pistache::Http::Header::ContentType>(MIME(Application, Json));

    // Handle OPTIONS requests (for CORS)
    if (request.method() == Pistache::Http::Method::Options) {
        response.send(Pistache::Http::Code::Ok, "");
        return;
    }
    
    // Handle GET requests
    if (request.method() == Pistache::Http::Method::Get) {
        auto uri = request.resource();
        
        // Parse max_samples query parameter if present
        size_t max_samples = 1000;  // Default
        if (request.query().has("max_samples")) {
            try {
                max_samples = std::stoul(request.query().get("max_samples").value());
            } catch (const std::exception& e) {
                // Invalid parameter, use default
            }
        }
        
        // Route: /data/all - Get all channel data
        if (uri == "/data/all") {
            auto data = generator_->getAllChannelsData(max_samples);
            response.send(Pistache::Http::Code::Ok, data.dump(2));
            return;
        }
        
        // Route: /data/{channel_name} - Get specific channel data
        static const std::string data_prefix = "/data/";
        if (uri.rfind(data_prefix, 0) == 0) {
            std::string channel_name = uri.substr(data_prefix.length());
            
            auto data = generator_->getChannelData(channel_name, max_samples);
            
            if (data.contains("error")) {
                response.send(Pistache::Http::Code::Not_Found, data.dump(2));
            } else {
                response.send(Pistache::Http::Code::Ok, data.dump(2));
            }
            return;
        }
        
        // Route: / - Home/index
        if (uri == "/") {
            nlohmann::json info = {
                {"service", "Mock Data API Server"},
                {"endpoints", {
                    {
                        {"path", "/data/all"},
                        {"description", "Get data from all channels"},
                        {"parameters", {
                            {"max_samples", "Maximum number of samples per channel (default: 1000)"}
                        }}
                    },
                    {
                        {"path", "/data/{channel_name}"},
                        {"description", "Get data for a specific channel"},
                        {"parameters", {
                            {"max_samples", "Maximum number of samples (default: 1000)"}
                        }}
                    }
                }}
            };
            
            response.send(Pistache::Http::Code::Ok, info.dump(2));
            return;
        }
        
        // Not found
        nlohmann::json error = {
            {"error", "Not found"},
            {"message", "The requested resource does not exist"}
        };
        response.send(Pistache::Http::Code::Not_Found, error.dump(2));
        return;
    }
    
    // Method not allowed
    nlohmann::json error = {
        {"error", "Method not allowed"},
        {"message", "Only GET requests are supported"}
    };
    response.send(Pistache::Http::Code::Method_Not_Allowed, error.dump(2));
}

// ---- MockDataAPIServerNode Implementation ----

MockDataAPIServerNode::MockDataAPIServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mock_data_api_server", options),
      server_(nullptr)
{
    RCLCPP_INFO(get_logger(), "Constructing MockDataAPIServerNode");
    
    // Create the data generator
    generator_ = std::make_shared<MockDataGenerator>();
    generator_->initialize();
    generator_->startGeneration();  // Start generating data at 10Hz
    
    // Initialize the server
    initServer();
}

MockDataAPIServerNode::~MockDataAPIServerNode()
{
    if (server_) {
        RCLCPP_INFO(get_logger(), "Shutting down Pistache server...");
        server_->shutdown();
        delete server_;
    }
    
    if (generator_) {
        generator_->stopGeneration();
    }
}

void MockDataAPIServerNode::initServer()
{
    // Listen on 0.0.0.0:9365
    Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(9365));
    
    auto opts = Pistache::Http::Endpoint::options()
        .threads(2);  // More threads for better performance
    
    // Create the server endpoint
    server_ = new Pistache::Http::Endpoint(addr);
    
    // Initialize and set up the request handler
    server_->init(opts);
    server_->setHandler(std::make_shared<MockDataAPIHandler>(generator_));
    server_->serveThreaded();
    
    RCLCPP_INFO(get_logger(), "Mock Data API Server listening on port 9365");
}

} // namespace mock_data_api