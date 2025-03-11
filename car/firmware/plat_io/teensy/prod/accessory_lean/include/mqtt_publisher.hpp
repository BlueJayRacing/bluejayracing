#pragma once

#include <Arduino.h>
#include <QNEthernet.h>
#include <PubSubClient.h>
#include <vector>
#include <string>
#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_channel_config.hpp"

namespace baja {
namespace network {

using namespace qindesign::network;

/**
 * @brief MQTT publisher for telemetry data
 * 
 * Handles publishing data to an MQTT broker over Ethernet.
 * Uses QNEthernet and PubSubClient libraries.
 */
class MQTTPublisher {
public:
    /**
     * @brief Construct a new MQTT Publisher
     * 
     * @param ringBuffer Reference to the ring buffer to read samples from
     */
    MQTTPublisher(buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer);
    
    /**
     * @brief Destroy the MQTT Publisher
     */
    ~MQTTPublisher();
    
    /**
     * @brief Initialize Ethernet and connect to MQTT broker
     * 
     * @param macAddress MAC address to use (if nullptr, use built-in MAC)
     * @param clientId MQTT client ID
     * @param brokerIP MQTT broker IP address
     * @param port MQTT broker port
     * @return true if initialization and connection successful
     */
    bool begin(uint8_t* macAddress, const char* clientId, 
               const char* brokerIP, uint16_t port = 1883);
    
    /**
     * @brief Set the downsample ratio
     * 
     * Only 1 in every downsampleRatio samples will be published.
     * 
     * @param ratio Downsample ratio (1 = no downsampling)
     */
    void setDownsampleRatio(uint8_t ratio);
    
    /**
     * @brief Set channel configurations
     * 
     * @param channelConfigs Vector of channel configurations
     */
    void setChannelConfigs(const std::vector<adc::ChannelConfig>& channelConfigs);
    
    /**
     * @brief Set the base topic for MQTT publications
     * 
     * @param baseTopic Base topic (e.g., "baja/data")
     */
    void setBaseTopic(const std::string& baseTopic);
    
    /**
     * @brief Process data from the ring buffer
     * 
     * This function should be called regularly from the thread.
     * It reads data from the ring buffer and publishes it to the MQTT broker.
     * 
     * @return Number of messages published
     */
    size_t process();
    
    /**
     * @brief Check if connected to the MQTT broker
     * 
     * @return true if connected
     */
    bool isConnected() const;
    
    /**
     * @brief Force reconnection to MQTT broker
     * 
     * @return true if reconnection successful
     */
    bool reconnect();

private:
    // Reference to the ring buffer
    buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer_;
    
    // Network objects
    EthernetClient ethClient_;
    PubSubClient mqttClient_;
    
    // Configuration
    char clientId_[32];     // Use fixed size arrays instead of std::string 
    char brokerIP_[16];
    uint16_t port_;
    
    // MAC address
    uint8_t macAddress_[6];
    
    // Base topic
    char baseTopic_[32];
    
    // Downsampling
    uint8_t downsampleRatio_;
    uint32_t sampleCounter_;
    
    // Last read position in the ring buffer
    size_t lastReadPosition_;
    
    // Channel configurations
    std::vector<adc::ChannelConfig> channelConfigs_;
    
    // Temporary buffer for MQTT message
    char messageBuffer_[data::MQTT_OPTIMAL_MESSAGE_SIZE];
    
    /**
     * @brief Build a JSON batch of samples
     * 
     * @param samples Array of samples
     * @param count Number of samples
     * @return true if successful
     */
    bool buildJsonBatch(const data::ChannelSample* samples, size_t count);
    
    /**
     * @brief Get channel name from index
     * 
     * @param channelIndex Channel index
     * @return Channel name or empty string if not found
     */
    std::string getChannelName(uint8_t channelIndex) const;
    
    /**
     * @brief Check and maintain MQTT connection
     * 
     * @return true if connected
     */
    bool checkConnection();
};

} // namespace network
} // namespace baja