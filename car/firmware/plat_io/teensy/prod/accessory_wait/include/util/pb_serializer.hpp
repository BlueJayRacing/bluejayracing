#pragma once

#include <Arduino.h>
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "baja_sample.pb.h"  // Generated from our .proto file
#include "config/config.hpp"
#include "pb_encode.h"
#include "pb_decode.h"
#include <vector>

namespace baja {
namespace serialization {

// Forward declaration
class SerializedBuffer;

/**
 * @brief Driver for Protocol Buffer serialization
 */
class PBSerializer {
public:
    /**
     * @brief Construct a new PBSerializer object
     * 
     * @param inputBuffer Reference to the ring buffer to read samples from
     * @param outputBuffer Reference to the buffer to write serialized messages to
     */
    PBSerializer(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& inputBuffer,
                SerializedBuffer& outputBuffer);
    
    /**
     * @brief Destroy the PBSerializer object
     */
    ~PBSerializer();
    
    /**
     * @brief Process samples from the input buffer and serialize them
     * 
     * @param maxSamples Maximum number of samples to process
     * @return Number of samples serialized
     */
    size_t processSamples(size_t maxSamples);
    
    /**
     * @brief Set the maximum timestamp delta
     * 
     * If a timestamp delta exceeds this value, a verbose chunk will be created
     * 
     * @param maxDelta Maximum timestamp delta in microseconds
     */
    void setMaxTimestampDelta(uint16_t maxDelta);
    
    /**
     * @brief Get serialization statistics
     * 
     * @return String with serialization statistics
     */
    String getStats() const;

private:
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& inputBuffer_;
    SerializedBuffer& outputBuffer_;
    
    uint16_t maxTimestampDelta_;  // Maximum timestamp delta for fixed chunks
    uint64_t lastBaseTimestamp_;  // Last base timestamp used
    size_t samplesProcessed_;     // Total samples processed
    size_t fixedChunksCreated_;   // Number of fixed data chunks created
    size_t verboseChunksCreated_; // Number of verbose data chunks created
    
    /**
     * @brief Create a fixed data chunk
     * 
     * @param samples Array of samples
     * @param count Number of samples
     * @param baseTimestamp Base timestamp
     * @return true if successful
     */
    bool createFixedDataChunk(const data::ChannelSample* samples, size_t count, uint64_t baseTimestamp);
    
    /**
     * @brief Create a verbose data chunk
     * 
     * @param samples Array of samples
     * @param count Number of samples
     * @return true if successful
     */
    bool createVerboseDataChunk(const data::ChannelSample* samples, size_t count);
    
    /**
     * @brief Check if samples can fit in a fixed data chunk
     * 
     * @param samples Array of samples
     * @param count Number of samples
     * @param baseTimestamp Base timestamp
     * @return true if samples can fit in a fixed data chunk
     */
    bool canUseFixedChunk(const data::ChannelSample* samples, size_t count, uint64_t baseTimestamp);
};

/**
 * @brief Buffer for serialized Protocol Buffer messages
 */
class SerializedBuffer {
public:
    /**
     * @brief Maximum buffer size in bytes
     */
    static constexpr size_t MAX_BUFFER_SIZE = 1024 * 256;  // 256KB buffer
    
    /**
     * @brief Maximum message size in bytes
     */
    static constexpr size_t MAX_MESSAGE_SIZE = 1500;  // 1.5KB per message (MTU size)
    
    /**
     * @brief Construct a new SerializedBuffer object
     */
    SerializedBuffer();
    
    /**
     * @brief Destroy the SerializedBuffer object
     */
    ~SerializedBuffer();
    
    /**
     * @brief Add a serialized message to the buffer
     * 
     * @param data Pointer to the data
     * @param size Size of the data
     * @return true if successful
     */
    bool addMessage(const uint8_t* data, size_t size);
    
    /**
     * @brief Get a message from the buffer
     * 
     * @param data Output buffer
     * @param maxSize Maximum size of the output buffer
     * @return Size of the message, 0 if no message available
     */
    size_t getMessage(uint8_t* data, size_t maxSize);
    
    /**
     * @brief Get the number of messages in the buffer
     * 
     * @return Number of messages
     */
    size_t messageCount() const;
    
    /**
     * @brief Get the used buffer space in bytes
     * 
     * @return Used buffer space in bytes
     */
    size_t bytesUsed() const;
    
    /**
     * @brief Get the free buffer space in bytes
     * 
     * @return Free buffer space in bytes
     */
    size_t bytesFree() const;
    
    /**
     * @brief Check if the buffer is empty
     * 
     * @return true if the buffer is empty
     */
    bool isEmpty() const;
    
    /**
     * @brief Check if the buffer is full
     * 
     * @return true if the buffer is full
     */
    bool isFull() const;
    
    /**
     * @brief Clear the buffer
     */
    void clear();

private:
    struct Message {
        uint8_t data[MAX_MESSAGE_SIZE];
        size_t size;
    };
    
    std::vector<Message> messages_;
    size_t bytesUsed_;
    mutable Threads::Mutex mutex_;
};

} // namespace serialization
} // namespace baja