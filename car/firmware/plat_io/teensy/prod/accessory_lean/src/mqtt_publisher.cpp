#include "mqtt_publisher.hpp"
#include <ArduinoJson.h>

namespace baja {
namespace network {

MQTTPublisher::MQTTPublisher(buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      mqttClient_(ethClient_),
      port_(1883),
      downsampleRatio_(1),
      sampleCounter_(0),
      lastReadPosition_(0) {
    // Default topics
    strncpy(baseTopic_, "baja/data", sizeof(baseTopic_) - 1);
    baseTopic_[sizeof(baseTopic_) - 1] = '\0';
}

MQTTPublisher::~MQTTPublisher() {
    // Disconnect from MQTT broker
    mqttClient_.disconnect();
}

bool MQTTPublisher::begin(uint8_t* macAddress, const char* clientId, 
                         const char* brokerIP, uint16_t port) {
    // Store configuration
    strncpy(clientId_, clientId, sizeof(clientId_) - 1);
    clientId_[sizeof(clientId_) - 1] = '\0';  // Ensure null termination
    
    strncpy(brokerIP_, brokerIP, sizeof(brokerIP_) - 1);
    brokerIP_[sizeof(brokerIP_) - 1] = '\0';  // Ensure null termination
    
    port_ = port;
    
    // Set up MAC address
    if (macAddress) {
        memcpy(macAddress_, macAddress, 6);
    } else {
        // Use built-in MAC address from Teensy
        uint8_t teensyMac[6];
        Ethernet.macAddress(teensyMac);
        memcpy(macAddress_, teensyMac, 6);
    }
    
    // Initialize Ethernet
    Ethernet.begin(macAddress_);
    
    // Wait for Ethernet to come up
    uint32_t startTime = millis();
    while (!Ethernet.linkStatus() && millis() - startTime < 5000) {
        delay(100);
    }
    
    if (!Ethernet.linkStatus()) {
        return false;
    }
    
    // Set up MQTT client
    mqttClient_.setServer(brokerIP_, port_);
    
    // Try to connect
    return reconnect();
}

void MQTTPublisher::setDownsampleRatio(uint8_t ratio) {
    if (ratio > 0 && ratio <= data::MAX_DOWNSAMPLE_RATIO) {
        downsampleRatio_ = ratio;
    }
}

void MQTTPublisher::setChannelConfigs(const std::vector<adc::ChannelConfig>& channelConfigs) {
    channelConfigs_ = channelConfigs;
}

void MQTTPublisher::setBaseTopic(const std::string& baseTopic) {
    strncpy(baseTopic_, baseTopic.c_str(), sizeof(baseTopic_) - 1);
    baseTopic_[sizeof(baseTopic_) - 1] = '\0';  // Ensure null termination
}

size_t MQTTPublisher::process() {
    // Check MQTT connection
    if (!checkConnection()) {
        return 0;
    }
    
    // Allow MQTT client to process incoming messages
    mqttClient_.loop();
    
    // Get available sample count
    size_t availableSamples = ringBuffer_.available();
    if (availableSamples == 0) {
        return 0;
    }
    
    // Determine how many samples to process
    // We need to peek into the buffer since we're not removing samples
    size_t maxSamples = data::MQTT_OPTIMAL_MESSAGE_SIZE / 50; // Rough estimate of JSON per sample
    size_t samplesToProcess = min(availableSamples, maxSamples);
    
    // Apply downsampling
    size_t downsampledCount = (samplesToProcess + downsampleRatio_ - 1) / downsampleRatio_;
    
    // Allocate temporary buffer for downsampled data
    data::ChannelSample* samples = new data::ChannelSample[downsampledCount];
    
    // Peek samples and apply downsampling
    size_t sampleIndex = 0;
    for (size_t i = 0; i < samplesToProcess; i++) {
        data::ChannelSample sample;
        if (ringBuffer_.peek(sample, i)) {
            if (sampleCounter_ % downsampleRatio_ == 0) {
                samples[sampleIndex++] = sample;
            }
            sampleCounter_++;
        }
    }
    
    // Build JSON batch and publish
    bool published = false;
    if (sampleIndex > 0) {
        if (buildJsonBatch(samples, sampleIndex)) {
            // Publish to MQTT
            char publishTopic[64];
            snprintf(publishTopic, sizeof(publishTopic), "%s/samples", baseTopic_);
            published = mqttClient_.publish(publishTopic, (char*)mqttMessageBuffer, strlen((char*)mqttMessageBuffer));
            
            // Update last read position
            lastReadPosition_ += samplesToProcess;
        }
    }
    
    // Free the temporary buffer
    delete[] samples;
    
    return published ? sampleIndex : 0;
}

bool MQTTPublisher::isConnected() const {
    // Need to cast away const since PubSubClient::connected() is not const
    return const_cast<PubSubClient&>(mqttClient_).connected();
}

bool MQTTPublisher::reconnect() {
    // Try to connect to MQTT broker
    char publishTopic[64];
    snprintf(publishTopic, sizeof(publishTopic), "%s/status", baseTopic_);
    
    if (mqttClient_.connect(clientId_)) {
        // Subscribe to command topic
        char subscribeTopic[64];
        snprintf(subscribeTopic, sizeof(subscribeTopic), "%s/command", baseTopic_);
        mqttClient_.subscribe(subscribeTopic);
        return true;
    }
    
    return false;
}

bool MQTTPublisher::buildJsonBatch(const data::ChannelSample* samples, size_t count) {
    // Create a JSON document
    // Create JSON document (with size based on MQTT message size)
    StaticJsonDocument<data::MQTT_OPTIMAL_MESSAGE_SIZE> doc;
    
    // Create samples array
    JsonArray samplesArray = doc.createNestedArray("samples");
    
    // Add each sample
    for (size_t i = 0; i < count; i++) {
        JsonObject sample = samplesArray.createNestedObject();
        sample["ts"] = samples[i].timestamp;
        sample["ch"] = samples[i].channelIndex;
        sample["val"] = samples[i].rawValue;
        
        // Add channel name if available
        std::string channelName = getChannelName(samples[i].channelIndex);
        if (!channelName.empty()) {
            sample["name"] = channelName;
        }
    }
    
    // Serialize to JSON
    size_t len = serializeJson(doc, messageBuffer_, data::MQTT_OPTIMAL_MESSAGE_SIZE);
    
    // Ensure null termination
    messageBuffer_[len] = '\0';
    
    return len > 0;
}

std::string MQTTPublisher::getChannelName(uint8_t channelIndex) const {
    for (const auto& config : channelConfigs_) {
        if (config.channelIndex == channelIndex) {
            return config.name;
        }
    }
    
    return "";
}

bool MQTTPublisher::checkConnection() {
    // Check Ethernet link
    if (!Ethernet.linkStatus()) {
        return false;
    }
    
    // Check MQTT connection
    if (!mqttClient_.connected()) {
        // Try to reconnect
        return reconnect();
    }
    
    return true;
}

} // namespace network
} // namespace baja