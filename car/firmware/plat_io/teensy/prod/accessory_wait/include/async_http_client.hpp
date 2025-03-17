#pragma once

#include <Arduino.h>
#include <QNEthernet.h>
#include <vector>
#include <string>
#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_channel_config.hpp"
#include "config.hpp"
#include "defines.h"
// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "AsyncHTTPRequest_Teensy41.hpp"


namespace baja {
namespace network {

using namespace qindesign::network;

/**
 * @brief Asynchronous HTTP client for telemetry data
 * 
 * Handles publishing data to a server via HTTP POST requests.
 * Uses AsyncHTTPRequest_Teensy41 library.
 */
class AsyncHTTPClient {
public:
    /**
     * @brief Construct a new Async HTTP Client
     * 
     * @param ringBuffer Reference to the ring buffer to read samples from
     */
    AsyncHTTPClient(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer);
    
    /**
     * @brief Destroy the Async HTTP Client
     */
    ~AsyncHTTPClient();
    
    /**
     * @brief Initialize Ethernet and HTTP client
     * 
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @param endpoint API endpoint to post to (e.g., "/api/data")
     * @return true if initialization successful
     */
    bool begin(const char* serverAddress, uint16_t port = 80, const char* endpoint = "/api/data");
    
    /**
     * @brief Initialize network connection with proper error handling
     * 
     * @return true if network initialized successfully
     */
    bool initializeNetwork();
    
    /**
     * @brief Convert IP address to string for logging
     * 
     * @param ip IP address to convert
     * @return String representation of the IP address
     */
    static String ipToString(IPAddress ip);
    
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
     * @param channelConfigs Pointer to channel configurations array
     */
    void setChannelConfigs(baja::adc::ChannelConfig* channelConfigs);
    
    /**
     * @brief Set custom HTTP header
     * 
     * @param headerName Header name
     * @param headerValue Header value
     */
    void setHeader(const char* headerName, const char* headerValue);
    
    /**
     * @brief Process data from the ring buffer
     * 
     * This function should be called regularly from the thread.
     * It reads data from the ring buffer and sends it to the HTTP server.
     * 
     * @return Number of samples processed
     */
    size_t process();
    
    /**
     * @brief Check if connected to the network
     * 
     * @return true if connected
     */
    bool isConnected() const;
    
    /**
     * @brief Attempt to reconnect to the network
     * 
     * @return true if reconnection successful
     */
    bool reconnect();

private:
    // Reference to the ring buffer
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer_;
    
    // HTTP request object (defined in the cpp file)
    AsyncHTTPRequest* requestPtr_;
    
    // Server configuration
    char serverAddress_[64];
    char endpoint_[32];
    uint16_t port_;
    
    // Custom headers
    struct Header {
        char name[32];
        char value[64];
    };
    std::vector<Header> customHeaders_;
    
    // Downsampling
    uint8_t downsampleRatio_;
    uint32_t sampleCounter_;
    
    // State tracking
    size_t lastReadPosition_;
    volatile bool requestInProgress_;
    uint32_t lastRequestTime_;
    int lastResponseCode_;
    uint32_t lastErrorReportTime_;  // Track time of last error report to reduce spam
    int lastReadyState_;
    bool networkInitialized_;      // Track if network was successfully initialized
    uint32_t retryCount_;          // Network reconnection attempt counter
    
    // Statistics
    uint32_t successCount_ = 0;
    uint32_t errorCount_ = 0;
    
    // Channel configurations
    baja::adc::ChannelConfig* channelConfigs_;
    
    // Buffer for message
    char messageBuffer_[data::MQTT_OPTIMAL_MESSAGE_SIZE];
    
    /**
     * @brief Build JSON batch of samples
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
     * @return std::string Channel name or empty string if not found
     */
    std::string getChannelName(uint8_t channelIndex) const;
    
    /**
     * @brief Send HTTP request using AsyncHTTPRequest library
     * 
     * @return true if request was initiated successfully
     */
    bool sendRequest();
    
    /**
     * @brief Send HTTP request using direct EthernetClient
     * 
     * @return true if request was initiated or is in progress
     */
    bool sendDirectRequest();
    
    /**
     * @brief Start a new direct HTTP request using the state machine
     * 
     * @return true if request was initiated successfully
     */
    bool startDirectRequest();
    
    /**
     * @brief Continue processing an ongoing HTTP request
     * 
     * @return true if request is in progress or completed successfully
     */
    bool continueDirectRequest();
    
    /**
     * @brief Static callback for HTTP request completion
     * 
     * This function will be called when the request state changes
     * 
     * @param optParm Optional parameter (this instance)
     * @param request HTTP request object
     * @param readyState Ready state
     */
    static void requestCallback(void* optParm, AsyncHTTPRequest* request, int readyState);
    
    /**
     * @brief Check network connection
     * 
     * @return true if connected
     */
    bool checkConnection();
};

} // namespace network
} // namespace baja