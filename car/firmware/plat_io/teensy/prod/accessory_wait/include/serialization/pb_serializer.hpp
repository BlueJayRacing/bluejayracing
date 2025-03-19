#pragma once

#include <Arduino.h>
#include <vector>
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include "baja_sample.pb.h"

namespace baja {
namespace serialization {

/**
 * @brief Encoded message structure for transmission
 */
struct EncodedMessage {
    uint8_t buffer[config::PB_MAX_MESSAGE_SIZE]; // Buffer for encoded message
    size_t size;                                // Actual size of the encoded message
    uint32_t timestamp;                         // Timestamp when the message was created
    
    EncodedMessage() : size(0), timestamp(0) {}
};

/**
 * @brief Serialize and encode samples into protobuf messages
 */
class PBSerializer {
public:
    /**
     * @brief Initialize the serializer
     * 
     * @param sourceBuffer Source buffer containing raw samples
     * @param targetBuffer Target buffer for encoded messages
     * @return true if initialization successful
     */
    static bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& sourceBuffer,
        buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& targetBuffer);
    
    /**
     * @brief Process a batch of samples and encode them
     * 
     * @param maxSamples Maximum number of samples to process
     * @return Number of samples processed
     */
    static size_t processBatch(size_t maxSamples = config::MAX_SAMPLES_PER_BATCH);
    
    /**
     * @brief Get the last read position in the source buffer
     * 
     * @return Last read position
     */
    static size_t getLastReadPosition();
    
    /**
     * @brief Reset the last read position in the source buffer
     */
    static void resetLastReadPosition();
    
    /**
     * @brief Get the number of encoded messages created
     * 
     * @return Encoded message count
     */
    static uint32_t getEncodedCount();
    
    /**
     * @brief Get the number of samples encoded
     * 
     * @return Encoded sample count
     */
    static uint32_t getSampleCount();
    
    /**
     * @brief Test encoding with test data (for diagnostics)
     * 
     * Creates test samples and attempts to encode them,
     * useful for debugging the encoding process.
     * 
     * @return true if test succeeds
     */
    static bool testEncoding();

private:
    static buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>* sourceBuffer_;
    static buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>* targetBuffer_;
    static size_t lastReadPosition_;
    static uint32_t encodedCount_;
    static uint32_t sampleCount_;
    
    /**
     * @brief Encode samples into a nanopb protobuf message
     * 
     * @param samples Array of sample pointers
     * @param count Number of samples
     * @param encodedMsg Output encoded message
     * @return true if encoding successful
     */
    static bool encodeSamples(
        const data::ChannelSample* samples, 
        size_t count, 
        EncodedMessage& encodedMsg);
};

} // namespace serialization
} // namespace baja