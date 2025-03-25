#pragma once

#include <TeensyThreads.h>
#include "pb_serializer.hpp"
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"

namespace baja {
namespace serialization {

/**
 * @brief Protobuf serialization thread module
 * 
 * Handles initialization, running, and monitoring of the protobuf serialization thread.
 */
class PBThread {
public:
    /**
     * @brief Initialize the PB thread module
     * 
     * @param sourceBuffer Source buffer containing raw samples
     * @param targetBuffer Target buffer for encoded messages
     * @return true if initialization was successful
     */
    static bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& sourceBuffer,
        buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& targetBuffer);
    
    /**
     * @brief Start the PB thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the PB thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the PB thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the serializer instance
     * 
     * @return Pointer to the PB serializer
     */
    static PBSerializer* getSerializer();
    
    /**
     * @brief Get encoding statistics
     * 
     * @param encodedCount Output parameter for number of encoded messages
     * @param sampleCount Output parameter for number of samples encoded
     */
    static void getStats(uint32_t& encodedCount, uint32_t& sampleCount);
    
    /**
     * @brief Get detailed timing statistics
     * 
     * @param avgProcessingTime Output parameter for average processing time in microseconds
     * @param minProcessingTime Output parameter for minimum processing time in microseconds
     * @param maxProcessingTime Output parameter for maximum processing time in microseconds
     * @param totalBatches Output parameter for total number of batches processed
     */
    static void getTimingStats(float& avgProcessingTime, uint32_t& minProcessingTime, 
                             uint32_t& maxProcessingTime, uint32_t& totalBatches);
    
    /**
     * @brief Print benchmark results to debug output
     */
    static void printBenchmarks();

private:
    static int threadId_;
    static volatile bool running_;
    static uint32_t lastProcessTime_;
    static uint32_t batchesProcessed_;
    static uint32_t samplesProcessed_;
    
    // Benchmarking metrics
    static uint32_t batchProcessingTimes_[100];  // Circular buffer of processing times
    static uint8_t timeIndex_;                  // Current index in circular buffer
    static uint32_t maxProcessingTime_;         // Maximum processing time recorded in microseconds
    static uint32_t minProcessingTime_;         // Minimum processing time recorded in microseconds
    static uint32_t totalProcessingTime_;       // Sum of all processing times for average calculation
    static uint32_t benchmarkStartTime_;        // Time when benchmarking started
    
    /**
     * @brief Record processing time for benchmarking
     * 
     * @param processingTime Processing time in microseconds
     */
    static void recordProcessingTime(uint32_t processingTime);
    
    /**
     * @brief PB serialization thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);
};

} // namespace serialization
} // namespace baja