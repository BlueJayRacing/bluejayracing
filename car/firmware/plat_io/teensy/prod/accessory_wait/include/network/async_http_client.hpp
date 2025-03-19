#pragma once

#include <Arduino.h>
#include <QNEthernet.h>
#include <vector>
#include <string>
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "adc/adc_channel_config.hpp"
#include "config/config.hpp"
#include "config/defines.h"
#include "serialization/pb_serializer.hpp"
// Required for AsyncHTTPRequest class
#include <AsyncHTTPRequest_Teensy41.hpp>

namespace baja {
namespace network {

using namespace qindesign::network;

// State machine states for HTTP client
enum HttpClientState {
    STATE_IDLE,
    STATE_CONNECTING,
    STATE_SENDING,
    STATE_WAITING,
    STATE_READING,
    STATE_PROCESSING,
    STATE_CLOSING,
    STATE_KEEPALIVE   // New state for persistent connections
};

/**
 * @brief Asynchronous HTTP client for telemetry data
 * 
 * Handles publishing data to a server via HTTP POST requests.
 * Uses a state machine approach with persistent connections.
 */
class AsyncHTTPClient {
public:
    /**
     * @brief Construct a new Async HTTP Client
     * 
     * @param encodedBuffer Reference to the buffer containing encoded protobuf messages
     */
    AsyncHTTPClient(buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer);
    
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
     * @param usePersistentConnection Whether to use persistent connections (keep-alive)
     * @return true if initialization successful
     */
    bool begin(const char* serverAddress, uint16_t port = 80, const char* endpoint = "/api/data", 
              bool usePersistentConnection = true);
    
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
     * @brief Process encoded messages from the buffer
     * 
     * This function should be called regularly from the thread.
     * It reads encoded messages from the buffer and sends them to the HTTP server.
     * 
     * @return Number of messages processed
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
    
    /**
     * @brief Set the persistent connection mode
     * 
     * @param enable Whether to use persistent connections
     */
    void setPersistentConnection(bool enable);
    
    /**
     * @brief Get current state name for debugging
     * 
     * @return String representation of current state
     */
    String getStateName() const;

private:
    // Reference to the encoded message buffer (instead of raw samples)
    buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer_;
    
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
    
    // Connection management
    bool usePersistentConnection_;
    bool connectionEstablished_;
    uint32_t idleConnectionTime_;
    uint32_t connectionAttempts_;
    
    // Downsampling
    uint8_t downsampleRatio_;
    uint32_t sampleCounter_;
    
    // State tracking
    volatile HttpClientState currentState_;
    uint32_t stateStartTime_;
    size_t lastReadPosition_;
    volatile bool requestInProgress_;
    uint32_t lastRequestTime_;
    int lastResponseCode_;
    uint32_t lastErrorReportTime_;  // Track time of last error report to reduce spam
    int lastReadyState_;
    bool networkInitialized_;      // Track if network was successfully initialized
    uint32_t retryCount_;          // Network reconnection attempt counter
    
    // State variables for direct request
    size_t bytesSent_;
    String responseBuffer_;
    int statusCode_;
    
    // Statistics
    uint32_t successCount_;
    uint32_t errorCount_;
    uint32_t timeoutCount_;
    uint32_t requestCount_;
    
    // Channel configurations
    baja::adc::ChannelConfig* channelConfigs_;
    
    // EthernetClient for persistent connection
    EthernetClient client_;
    
    // Dynamic buffer for message (instead of fixed size)
    char* messageBuffer_;
    size_t messageBufferSize_;
    
    /**
     * @brief Build JSON wrapper for encoded protobuf message
     * 
     * @param encodedMsg Encoded protobuf message
     * @return true if successful
     */
    bool buildJsonWrapper(const serialization::EncodedMessage& encodedMsg);
    
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
     * @brief Close the connection if needed
     * 
     * @param force Force close even if using persistent connections
     */
    void closeConnection(bool force = false);
    
    /**
     * @brief Establish a connection to the server if not already connected
     * 
     * @return true if connection is established or in progress
     */
    bool ensureConnection();
    
    /**
     * @brief Manage persistent connection
     * 
     * Checks if connection is still alive and resets if idle too long
     * 
     * @return true if connection is healthy
     */
    bool managePersistentConnection();
    
    /**
     * @brief Process a completed response
     * 
     * @return true if processing was successful
     */
    bool processResponse();
    
    /**
     * @brief Reset the state machine for the next request
     * 
     * @param closeConn Whether to close the connection
     */
    void resetState(bool closeConn = false);
    
    /**
     * @brief Log timeout error with appropriate details
     */
    void logTimeoutError();
    
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