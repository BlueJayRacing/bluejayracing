#include "async_http_client.hpp"
#include <ArduinoJson.h>
#include "debug_util.hpp"


namespace baja {
namespace network {

AsyncHTTPClient::AsyncHTTPClient(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      requestPtr_(nullptr),
      port_(80),
      downsampleRatio_(1),
      sampleCounter_(0),
      lastReadPosition_(0),
      requestInProgress_(false),
      lastRequestTime_(0),
      lastResponseCode_(0),
      lastReadyState_(0),
      channelConfigs_(nullptr) {
    
    // Initialize default server address
    strncpy(serverAddress_, "localhost", sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    
    // Initialize default endpoint
    strncpy(endpoint_, "/api/data", sizeof(endpoint_) - 1);
    endpoint_[sizeof(endpoint_) - 1] = '\0';
    
    // Create the request object
    requestPtr_ = new AsyncHTTPRequest();
    
    if (!requestPtr_) {
        util::Debug::error("Failed to create AsyncHTTPRequest object");
    }
}

AsyncHTTPClient::~AsyncHTTPClient() {
    // Clean up the request object
    if (requestPtr_) {
        delete requestPtr_;
        requestPtr_ = nullptr;
    }
}

bool AsyncHTTPClient::begin(const char* serverAddress, uint16_t port, const char* endpoint) {
    // Store server configuration
    strncpy(serverAddress_, serverAddress, sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    
    strncpy(endpoint_, endpoint, sizeof(endpoint_) - 1);
    endpoint_[sizeof(endpoint_) - 1] = '\0';
    
    port_ = port;
    
    util::Debug::info("Initializing AsyncHTTPClient...");
    
    // Make sure Ethernet is initialized (should be done by main.cpp)
    // We'll just verify connection status here
    if (!checkConnection()) {
        util::Debug::warning("Network not connected, will try to reconnect later");
    }
    
    // Set up request callback
    AsyncHTTPRequest* request = static_cast<AsyncHTTPRequest*>(requestPtr_);
    request->setTimeout(DEFAULT_RX_TIMEOUT); // Use timeout from defines.h
    request->setDebug(false); // Disable verbose library debugging
    
    // Set the callback for state changes
    request->onReadyStateChange([this](void* arg, AsyncHTTPRequest* req, int readyState) {
        this->requestCallback(arg, req, readyState);
    }, this);
    
    util::Debug::info("AsyncHTTPClient initialized successfully");
    util::Debug::info("Server: " + String(serverAddress_) + ":" + String(port_) + endpoint_);
    
    return true;
}

void AsyncHTTPClient::setDownsampleRatio(uint8_t ratio) {
    if (ratio > 0) {
        downsampleRatio_ = ratio;
        util::Debug::detail("Downsample ratio set to " + String(downsampleRatio_));
    }
}

void AsyncHTTPClient::setChannelConfigs(baja::adc::ChannelConfig* channelConfigs) {
    channelConfigs_ = channelConfigs;
    util::Debug::detail("Channel configurations set");
}

void AsyncHTTPClient::setHeader(const char* headerName, const char* headerValue) {
    // Add or update header
    for (auto& header : customHeaders_) {
        if (strcmp(header.name, headerName) == 0) {
            strncpy(header.value, headerValue, sizeof(header.value) - 1);
            header.value[sizeof(header.value) - 1] = '\0';
            return;
        }
    }
    
    // Add new header
    Header newHeader;
    strncpy(newHeader.name, headerName, sizeof(newHeader.name) - 1);
    newHeader.name[sizeof(newHeader.name) - 1] = '\0';
    
    strncpy(newHeader.value, headerValue, sizeof(newHeader.value) - 1);
    newHeader.value[sizeof(newHeader.value) - 1] = '\0';
    
    customHeaders_.push_back(newHeader);
    util::Debug::detail("Added header: " + String(headerName) + ": " + String(headerValue));
}

size_t AsyncHTTPClient::process() {
    // If a request is already in progress, return
    if (requestInProgress_) {
        return 0;
    }
    
    // Check if enough time has passed since last request (rate limiting)
    uint32_t currentTime = millis();
    if (currentTime - lastRequestTime_ < 1000) { // At least 1 second between requests
        return 0;
    }
    
    // Check network connection
    if (!isConnected()) {
        util::Debug::detail("Network not connected, skipping HTTP request");
        return 0;
    }
    
    // Get number of available samples in the ring buffer
    size_t availableSamples = ringBuffer_.available();
    if (availableSamples == 0) {
        return 0;
    }
    
    // Determine how many samples to process (limit to a reasonable batch size)
    size_t maxSamples = 100; // Modest limit to keep JSON size reasonable
    size_t samplesToProcess = min(availableSamples, maxSamples);
    
    util::Debug::detail("Processing " + String(samplesToProcess) + " samples out of " + 
                     String(availableSamples) + " available");
    
    // Create a temporary buffer for the samples we'll process
    data::ChannelSample* samples = new data::ChannelSample[samplesToProcess];
    if (!samples) {
        util::Debug::error("Failed to allocate sample buffer");
        return 0;
    }
    
    // Peek samples from the ring buffer (don't remove them yet)
    size_t sampleIndex = 0;
    for (size_t i = 0; i < samplesToProcess; i++) {
        data::ChannelSample sample;
        if (ringBuffer_.peek(sample, i + lastReadPosition_)) {
            if (sampleCounter_++ % downsampleRatio_ == 0) {
                samples[sampleIndex++] = sample;
            }
        }
    }
    
    util::Debug::detail("After downsampling, sending " + String(sampleIndex) + " samples");
    
    // Build the JSON batch for the samples
    size_t processedCount = 0;
    if (sampleIndex > 0) {
        if (buildJsonBatch(samples, sampleIndex)) {
            // Send HTTP request with the JSON data
            if (sendRequest()) {
                // Mark the samples as processed
                lastReadPosition_ += samplesToProcess;
                processedCount = sampleIndex;
                
                // Start rate limiting timer
                lastRequestTime_ = currentTime;
                requestInProgress_ = true;
                
                util::Debug::info("HTTP request sent with " + String(sampleIndex) + " samples");
            }
        }
    }
    
    // Free the temporary buffer
    delete[] samples;
    
    return processedCount;
}

bool AsyncHTTPClient::isConnected() const {
    return Ethernet.linkStatus() == LinkStatus_kLinkStatusUp;
}

bool AsyncHTTPClient::reconnect() {
    util::Debug::info("Attempting to reconnect to network...");
    
    // Try to re-initialize Ethernet
    Ethernet.begin();
    
    // Wait up to 5 seconds for link
    uint32_t startTime = millis();
    while (millis() - startTime < 5000) {
        if (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp) {
            util::Debug::info("Network reconnection successful");
            return true;
        }
        delay(100);
    }
    
    util::Debug::error("Network reconnection failed");
    return false;
}

bool AsyncHTTPClient::buildJsonBatch(const data::ChannelSample* samples, size_t count) {
    // Create a simplified JSON structure - cleaner to avoid potential issues
    DynamicJsonDocument doc(data::MQTT_OPTIMAL_MESSAGE_SIZE);
    
    // Add basic device info
    doc["device"] = "Teensy41";
    doc["timestamp"] = millis();
    doc["count"] = count;
    
    // Create an array for the samples
    JsonArray samplesArray = doc.createNestedArray("samples");
    
    // Add only essential sample data to avoid corruption
    for (size_t i = 0; i < count; i++) {
        JsonObject sample = samplesArray.createNestedObject();
        sample["ts"] = samples[i].timestamp;
        sample["ch"] = samples[i].channelIndex;
        sample["val"] = samples[i].rawValue;
        
        // Only add channel name if it's valid (no null or corrupted strings)
        if (channelConfigs_ && samples[i].channelIndex < adc::ADC_CHANNEL_COUNT) {
            const std::string& name = channelConfigs_[samples[i].channelIndex].name;
            if (!name.empty()) {
                // Validate the string - only include ASCII characters
                bool valid = true;
                for (char c : name) {
                    if (c == 0 || !isprint(c)) {
                        valid = false;
                        break;
                    }
                }
                
                if (valid) {
                    sample["name"] = name.c_str();
                }
            }
        }
    }
    
    // Serialize the JSON document to the message buffer
    size_t len = serializeJson(doc, messageBuffer_, data::MQTT_OPTIMAL_MESSAGE_SIZE);
    
    // Ensure null termination
    if (len < data::MQTT_OPTIMAL_MESSAGE_SIZE) {
        messageBuffer_[len] = '\0';
    } else {
        messageBuffer_[data::MQTT_OPTIMAL_MESSAGE_SIZE - 1] = '\0';
        util::Debug::warning("JSON message truncated");
    }
    
    // Log the full JSON for debugging
    util::Debug::detail("JSON payload: " + String(messageBuffer_));
    
    return len > 0;
}

std::string AsyncHTTPClient::getChannelName(uint8_t channelIndex) const {
    // Check if channel configs are available and valid
    if (channelConfigs_ && channelIndex < adc::ADC_CHANNEL_COUNT) {
        return channelConfigs_[channelIndex].name;
    }
    
    return ""; // Return empty string if no name available
}

bool AsyncHTTPClient::sendRequest() {
    AsyncHTTPRequest* request = static_cast<AsyncHTTPRequest*>(requestPtr_);
    
    // Check if the request object is ready for a new request
    if (request->readyState() != readyStateUnsent && request->readyState() != readyStateDone) {
        util::Debug::detail("HTTP request already in progress");
        return false;
    }
    
    // Format the URL for the request - use a simple approach like the example
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%d%s", serverAddress_, port_, endpoint_);
    
    util::Debug::detail("Opening HTTP request to: " + String(url));
    
    // Open the request
    if (!request->open("POST", url)) {
        util::Debug::error("Failed to open HTTP request");
        return false;
    }
    
    // Set minimal headers - don't use Content-Type to mimic the example
    request->setReqHeader("Connection", "keep-alive");
    
    // Add custom headers
    for (const auto& header : customHeaders_) {
        request->setReqHeader(header.name, header.value);
    }
    
    // Send the request with the JSON payload as raw string
    if (!request->send(messageBuffer_)) {
        util::Debug::error("Failed to send HTTP request");
        return false;
    }
    
    util::Debug::detail("HTTP request sent successfully");
    return true;
}

void AsyncHTTPClient::requestCallback(void* optParm, AsyncHTTPRequest* request, int readyState) {
    // Cast the optParm back to the instance
    AsyncHTTPClient* client = static_cast<AsyncHTTPClient*>(optParm);
    if (!client) {
        return;
    }
    
    // Log state changes for debugging
    if (readyState != client->lastReadyState_) {
        util::Debug::detail("HTTP request state changed: " + String(readyState));
        client->lastReadyState_ = readyState;
    }
    
    // Check if the request is complete
    if (readyState == readyStateDone) {
        int responseCode = request->responseHTTPcode();
        client->lastResponseCode_ = responseCode;
        
        // Log the response
        if (responseCode == 200) {
            util::Debug::info("HTTP request successful, code: " + String(responseCode));
            
            // Log the response for debugging
            String response = request->responseText();
            if (response.length() > 0) {
                // Log full response for debugging, truncate if very long
                if (response.length() > 200) {
                    util::Debug::detail("Response (truncated): " + response.substring(0, 200) + "...");
                } else {
                    util::Debug::detail("Response: " + response);
                }
            } else {
                util::Debug::detail("Empty response received");
            }
        } else {
            util::Debug::warning("HTTP request failed, code: " + String(responseCode));
            
            // For error responses, always log the full response if available
            String response = request->responseText();
            if (response.length() > 0) {
                util::Debug::detail("Error response: " + response);
            } else {
                // Log the HTTP status message when there's no response body
                util::Debug::detail("Error message: " + request->responseHTTPString());
            }
        }
        
        // Mark request as complete
        client->requestInProgress_ = false;
    }
}

bool AsyncHTTPClient::checkConnection() {
    // Check if Ethernet link is up
    if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
        util::Debug::info("Network link down, trying to reconnect...");
        
        // Try to reconnect
        Ethernet.begin();
        
        // Wait briefly for connection
        delay(500);
        
        // Check link status again
        if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
            return false;
        }
    }
    
    return true;
}

} // namespace network
} // namespace baja