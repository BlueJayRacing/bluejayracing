#pragma once

#include "network/pbudp_handler.hpp"
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"

namespace baja {
namespace network {

/**
 * @brief Combined Protocol Buffer serialization and UDP transmission module
 * 
 * Provides initialization, serialization, and UDP transmission for network data.
 */
namespace functions {

    /**
     * @brief Initialize the PBUDP module
     * 
     * @param sourceBuffer Fast path buffer containing samples to process
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @return true if initialization was successful
     */
    bool initialize(
        buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer,
        const char* serverAddress,
        uint16_t port = 8888);
    
    /**
     * @brief Start PBUDP operations
     * 
     * @return true if successful
     */
    bool start();
    
    /**
     * @brief Stop PBUDP operations
     * 
     * @return true if successful
     */
    bool stop();
    
    /**
     * @brief Check if PBUDP is running
     * 
     * @return true if running
     */
    bool isRunning();
    
    /**
     * @brief Process and send samples via UDP - called from master loop
     * 
     * Checks if there are enough samples (>= 50) before attempting to send.
     * 
     * @return Number of samples sent (0 if no sending occurred)
     */
    size_t process();
    
    /**
     * @brief Get the PBUDP handler instance
     * 
     * @return Pointer to the PBUDP handler
     */
    PBUDPHandler* getHandler();
    
    /**
     * @brief Get statistics about PBUDP operation
     * 
     * @param messagesSent Output parameter for number of messages sent
     * @param sampleCount Output parameter for number of samples processed
     * @param bytesTransferred Output parameter for number of bytes transferred
     * @param sendErrors Output parameter for number of send errors
     */
    void getStats(uint32_t& messagesSent, uint32_t& sampleCount, 
                uint32_t& bytesTransferred, uint32_t& sendErrors);
    
    /**
     * @brief Get timing statistics for PBUDP processing
     * 
     * @param avgTime Average processing time in microseconds
     * @param minTime Minimum processing time in microseconds
     * @param maxTime Maximum processing time in microseconds
     * @param messageCount Total messages sent
     */
    void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint32_t& messageCount);
    
    /**
     * @brief Reset timing statistics
     */
    void resetTimingStats();

} // namespace functions

} // namespace network
} // namespace baja