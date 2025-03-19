#pragma once

#include <TeensyThreads.h>
#include "network/async_http_client.hpp" 
#include "serialization/pb_serializer.hpp"
// All other includes are already in async_http_client.hpp

#include <QNEthernet.h>
#include <AsyncHTTPRequest_Teensy41.hpp> // Include this directly to ensure it's available


namespace baja {
namespace network {

/**
 * @brief HTTP client thread module
 * 
 * Handles initialization, running, and monitoring of the HTTP client thread.
 */
class HTTPThread {
public:
    /**
     * @brief Initialize the HTTP thread module
     * 
     * @param encodedBuffer Reference to the buffer containing encoded protobuf messages
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @param endpoint API endpoint to post to (e.g., "/api/data")
     * @return true if initialization was successful
     */
    static bool initialize(
        buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer,
        const char* serverAddress,
        uint16_t port = 80,
        const char* endpoint = "/api/data");
    
    /**
     * @brief Start the HTTP thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the HTTP thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the HTTP thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the HTTP client instance
     * 
     * @return Pointer to the HTTP client
     */
    static AsyncHTTPClient* getClient();
    
    /**
     * @brief Set channel configurations for the HTTP client
     * 
     * @param channelConfigs Pointer to channel configurations array
     */
    static void setChannelConfigs(adc::ChannelConfig* channelConfigs);
    
    /**
     * @brief Set the downsample ratio
     * 
     * @param ratio Downsample ratio (1 = no downsampling)
     */
    static void setDownsampleRatio(uint8_t ratio);

private:
    static AsyncHTTPClient* httpClient_;
    static int threadId_;
    static volatile bool running_;
    static uint32_t requestsCount_;
    static uint32_t successCount_;
    static uint32_t errorCount_;
    
    /**
     * @brief HTTP client thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);
};

} // namespace network
} // namespace baja