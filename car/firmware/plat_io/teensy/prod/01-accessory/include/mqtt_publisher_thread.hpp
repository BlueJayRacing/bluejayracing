#pragma once

#include <Arduino.h>
#include <TeensyThreads.h>
#include <QNEthernet.h>
#include <PubSubClient.h>
#include "data_buffer.hpp"

namespace baja {
namespace mqtt {

/**
 * @brief MQTT Publisher Thread
 * 
 * Thread responsible for publishing downsampled data to MQTT broker
 * with configurable downsampling ratio.
 */
class MQTTPublisherThread {
public:
    /**
     * @brief Construct a new MQTT Publisher Thread
     * 
     * @param dataBuffer Reference to the data buffer
     * @param client Reference to PubSubClient instance
     * @param topicPrefix MQTT topic prefix
     */
    MQTTPublisherThread(data::DataBuffer& dataBuffer, 
                      PubSubClient& client,
                      const char* topicPrefix);
    
    /**
     * @brief Destroy the MQTT Publisher Thread
     */
    ~MQTTPublisherThread();

    /**
     * @brief Start the MQTT publisher thread
     * 
     * @return int Thread ID or negative value on error
     */
    int start();
    
    /**
     * @brief Stop the MQTT publisher thread
     */
    void stop();
    
    /**
     * @brief Check if the thread is running
     * 
     * @return true if running
     */
    bool isRunning() const;
    
    /**
     * @brief Set the downsampling ratio
     * 
     * @param ratio Downsampling ratio (1=no downsampling, >1=skip samples)
     */
    void setDownsamplingRatio(uint8_t ratio);
    
    /**
     * @brief Get the current downsampling ratio
     * 
     * @return uint8_t Current ratio
     */
    uint8_t getDownsamplingRatio() const;
    
    /**
     * @brief Get the number of messages sent
     * 
     * @return uint32_t Message count
     */
    uint32_t getMessageCount() const;
    
    /**
     * @brief Get the error count
     * 
     * @return uint32_t Error count
     */
    uint32_t getErrorCount() const;
    
    /**
     * @brief Set the publish interval
     * 
     * @param intervalMs Interval in milliseconds
     */
    void setPublishInterval(unsigned long intervalMs);

private:
    /**
     * @brief Main thread function
     */
    static void threadFunction(void* arg);
    
    /**
     * @brief Format and publish a batch of samples
     * 
     * @param samples Pointer to sample buffer
     * @param count Number of samples
     * @return true if successful
     */
    bool publishSamples(const data::ChannelSample* samples, size_t count);
    
    /**
     * @brief Check and reconnect to MQTT broker if needed
     * 
     * @return true if connected
     */
    bool ensureConnected();

private:
    // Data buffer reference
    data::DataBuffer& dataBuffer_;
    
    // MQTT client
    PubSubClient& mqttClient_;
    
    // MQTT configuration
    char topicPrefix_[64];
    
    // Thread control
    int threadId_;
    volatile bool running_;
    
    // Downsampling configuration
    std::atomic<uint8_t> downsamplingRatio_;
    
    // Publishing settings
    std::atomic<unsigned long> publishIntervalMs_;
    
    // Statistics
    std::atomic<uint32_t> messageCount_;
    std::atomic<uint32_t> errorCount_;
    size_t lastSampleIndex_;
    
    // Data buffer for publishing
    data::ChannelSample* publishBuffer_;
    
    // Prevent copying
    MQTTPublisherThread(const MQTTPublisherThread&) = delete;
    MQTTPublisherThread& operator=(const MQTTPublisherThread&) = delete;
};

} // namespace mqtt
} // namespace baja