#include "mqtt_publisher_thread.hpp"
#include <Arduino.h>

namespace baja {
namespace mqtt {

MQTTPublisherThread::MQTTPublisherThread(data::DataBuffer& dataBuffer, 
                                       PubSubClient& client,
                                       const char* topicPrefix)
    : dataBuffer_(dataBuffer)
    , mqttClient_(client)
    , threadId_(-1)
    , running_(false)
    , downsamplingRatio_(10)  // Default: downsample by factor of 10
    , publishIntervalMs_(1000) // Default: publish every second
    , messageCount_(0)
    , errorCount_(0)
    , lastSampleIndex_(0)
{
    // Copy topic prefix
    strncpy(topicPrefix_, topicPrefix, sizeof(topicPrefix_) - 1);
    topicPrefix_[sizeof(topicPrefix_) - 1] = '\0'; // Ensure null termination
    
    // Allocate publish buffer - sized to fit optimal MQTT message size
    publishBuffer_ = new data::ChannelSample[data::MQTT_OPTIMAL_MESSAGE_SIZE / sizeof(data::ChannelSample)];
}

MQTTPublisherThread::~MQTTPublisherThread() {
    stop();
    
    // Free publish buffer
    if (publishBuffer_) {
        delete[] publishBuffer_;
        publishBuffer_ = nullptr;
    }
}

int MQTTPublisherThread::start() {
    if (isRunning()) {
        return threadId_; // Already running
    }
    
    // Start thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, this, 8 * 1024); // 8KB stack
    
    if (threadId_ < 0) {
        Serial.println("Failed to create MQTT publisher thread!");
        running_ = false;
        return -1;
    }
    
    Serial.print("MQTT publisher thread started with ID: ");
    Serial.println(threadId_);
    
    return threadId_;
}

void MQTTPublisherThread::stop() {
    if (isRunning()) {
        running_ = false;
        
        // Wait for thread to exit
        threads.wait(threadId_, 2000); // Wait up to 2 seconds
        
        threadId_ = -1;
        
        Serial.println("MQTT publisher thread stopped");
    }
}

bool MQTTPublisherThread::isRunning() const {
    return running_ && threadId_ >= 0;
}

void MQTTPublisherThread::setDownsamplingRatio(uint8_t ratio) {
    // Set a safe limit to prevent division by zero
    if (ratio < 1) ratio = 1;
    if (ratio > data::MAX_DOWNSAMPLE_RATIO) ratio = data::MAX_DOWNSAMPLE_RATIO;
    
    downsamplingRatio_.store(ratio);
    
    // Also update the buffer's downsampling ratio
    dataBuffer_.setDownsamplingRatio(ratio);
}

uint8_t MQTTPublisherThread::getDownsamplingRatio() const {
    return downsamplingRatio_.load();
}

uint32_t MQTTPublisherThread::getMessageCount() const {
    return messageCount_.load();
}

uint32_t MQTTPublisherThread::getErrorCount() const {
    return errorCount_.load();
}

void MQTTPublisherThread::setPublishInterval(unsigned long intervalMs) {
    // Set a reasonable minimum to prevent flooding
    if (intervalMs < 100) intervalMs = 100;
    
    publishIntervalMs_.store(intervalMs);
}

void MQTTPublisherThread::threadFunction(void* arg) {
    MQTTPublisherThread* self = static_cast<MQTTPublisherThread*>(arg);
    
    Serial.println("MQTT publisher thread function started");
    
    // Main loop - publish data at configured interval
    unsigned long lastPublishTime = millis();
    
    // Calculate buffer size in samples
    size_t maxSamples = data::MQTT_OPTIMAL_MESSAGE_SIZE / sizeof(data::ChannelSample);
    
    while (self->running_) {
        unsigned long currentTime = millis();
        
        // Check if it's time to publish
        if (currentTime - lastPublishTime >= self->publishIntervalMs_.load()) {
            lastPublishTime = currentTime;
            
            // Try to establish connection if not connected
            if (self->ensureConnected()) {
                // Get downsampled data from buffer
                auto [samplesRetrieved, nextIndex] = self->dataBuffer_.getDownsampledData(
                    self->downsamplingRatio_.load(),
                    self->lastSampleIndex_,
                    self->publishBuffer_,
                    maxSamples
                );
                
                // Update last sample index
                self->lastSampleIndex_ = nextIndex;
                
                // If samples were retrieved, publish them
                if (samplesRetrieved > 0) {
                    bool success = self->publishSamples(self->publishBuffer_, samplesRetrieved);
                    
                    if (success) {
                        self->messageCount_++;
                    } else {
                        self->errorCount_++;
                    }
                }
            } else {
                self->errorCount_++;
            }
        }
        
        // Let MQTT client process incoming messages
        self->mqttClient_.loop();
        
        // Yield to other threads
        threads.yield();
        delay(1); // Small delay to prevent tight loop
    }
    
    Serial.println("MQTT publisher thread function exiting");
}

bool MQTTPublisherThread::ensureConnected() {
    // If already connected, just return
    if (mqttClient_.connected()) {
        return true;
    }
    
    // Try to connect with a unique client ID
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "teensy-baja-%lu", millis());
    
    Serial.print("Attempting MQTT connection as ");
    Serial.print(clientId);
    Serial.print("...");
    
    // Try to connect with a clean session
    if (mqttClient_.connect(clientId)) {
        Serial.println("connected");
        
        // Optionally subscribe to control topics here
        // mqttClient_.subscribe("baja/control/#");
        
        return true;
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient_.state());
        Serial.println(" will try again later");
        
        return false;
    }
}

bool MQTTPublisherThread::publishSamples(const data::ChannelSample* samples, size_t count) {
    if (count == 0 || !mqttClient_.connected()) {
        return false;
    }
    
    // Prepare JSON buffer for samples
    // We'll use a simple binary format to maximize efficiency
    // Each topic will get data for a specific channel
    
    // Group samples by channel
    struct ChannelData {
        bool hasData;
        uint8_t channelIndex;
        uint64_t timestamps[16]; // Small buffer per channel
        uint32_t values[16];
        uint8_t count;
    };
    
    // Initialize channel data arrays
    ChannelData channelData[static_cast<int>(baja::adc::ChannelName::NUM_CHANNELS)];
    for (int i = 0; i < static_cast<int>(baja::adc::ChannelName::NUM_CHANNELS); i++) {
        channelData[i].hasData = false;
        channelData[i].channelIndex = i;
        channelData[i].count = 0;
    }
    
    // Group samples by channel
    for (size_t i = 0; i < count; i++) {
        uint8_t channelIndex = samples[i].channelIndex;
        
        if (channelIndex < static_cast<int>(baja::adc::ChannelName::NUM_CHANNELS)) {
            ChannelData& cd = channelData[channelIndex];
            
            // If we have space in this channel's buffer
            if (cd.count < 16) {
                cd.timestamps[cd.count] = samples[i].timestamp;
                cd.values[cd.count] = samples[i].rawValue;
                cd.count++;
                cd.hasData = true;
            }
        }
    }
    
    // Publish each channel with data
    bool allSucceeded = true;
    
    for (int i = 0; i < static_cast<int>(baja::adc::ChannelName::NUM_CHANNELS); i++) {
        if (channelData[i].hasData) {
            // Create topic name for this channel
            char topic[128];
            snprintf(topic, sizeof(topic), "%s/channel/%d", topicPrefix_, i);
            
            // Create a compact binary message
            // Format: [count(1)][timestamp(8)value(4)timestamp(8)value(4)...]
            uint8_t buffer[1 + (8 + 4) * 16]; // Max 16 samples per message
            
            // First byte is count
            buffer[0] = channelData[i].count;
            
            // Fill in timestamps and values
            for (int j = 0; j < channelData[i].count; j++) {
                // Copy timestamp (8 bytes)
                uint64_t ts = channelData[i].timestamps[j];
                memcpy(&buffer[1 + j * 12], &ts, 8);
                
                // Copy value (4 bytes)
                uint32_t val = channelData[i].values[j];
                memcpy(&buffer[1 + j * 12 + 8], &val, 4);
            }
            
            // Calculate message size
            size_t messageSize = 1 + channelData[i].count * 12;
            
            // Publish the message
            bool success = mqttClient_.publish(topic, buffer, messageSize);
            
            if (!success) {
                allSucceeded = false;
                
                // Log error only occasionally to avoid flooding
                if (errorCount_ % 10 == 0) {
                    Serial.print("Failed to publish to topic: ");
                    Serial.println(topic);
                }
            }
        }
    }
    
    return allSucceeded;
}

} // namespace mqtt
} // namespace baja