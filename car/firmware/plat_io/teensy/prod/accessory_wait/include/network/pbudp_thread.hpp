#pragma once

#include <TeensyThreads.h>
#include "network/pbudp_handler.hpp"
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"

namespace baja {
namespace network {

/**
 * @brief Combined Protocol Buffer serialization and UDP transmission thread
 * 
 * This class combines the functionality of the previous PB and UDP threads
 * into a single thread that efficiently processes samples, encodes them, and
 * sends them via UDP without intermediate buffers or thread switching.
 */
class PBUDPThread {
public:
    /**
     * @brief Initialize the combined PB+UDP thread
     * 
     * @param sourceBuffer Fast path buffer containing samples to process
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @return true if initialization was successful
     */
    static bool initialize(
        buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer,
        const char* serverAddress,
        uint16_t port = 8888);
    
    /**
     * @brief Start the combined thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the combined thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the combined thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the handler instance
     * 
     * @return Pointer to the combined handler
     */
    static PBUDPHandler* getHandler();
    
    /**
     * @brief Get statistics about combined thread operation
     * 
     * @param messagesSent Output parameter for number of messages sent
     * @param sampleCount Output parameter for number of samples processed
     * @param bytesTransferred Output parameter for number of bytes transferred
     * @param sendErrors Output parameter for number of send errors
     */
    static void getStats(uint32_t& messagesSent, uint32_t& sampleCount, 
                        uint32_t& bytesTransferred, uint32_t& sendErrors);

private:
    static PBUDPHandler* handler_;
    static int threadId_;
    static volatile bool running_;
    
    // Benchmark metrics
    static uint32_t processingTimes_[100];
    static uint8_t timeIndex_;
    static uint32_t maxProcessingTime_;
    static uint32_t minProcessingTime_;
    static uint32_t totalProcessingTime_;
    static uint32_t totalBatches_;
    static uint32_t benchmarkStartTime_;
    
    /**
     * @brief Combined thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);
    
    /**
     * @brief Record processing time for benchmarking
     * 
     * @param processingTime Processing time in microseconds
     */
    static void recordProcessingTime(uint32_t processingTime);
    
    /**
     * @brief Print benchmark results
     */
    static void printBenchmarks();
};

} // namespace network
} // namespace baja