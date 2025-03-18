#pragma once

#include <TeensyThreads.h>
#include "serialization/pb_serializer.hpp"
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"

namespace baja {
namespace serialization {

/**
 * @brief Protocol Buffer serialization thread module
 * 
 * Handles initialization, running, and monitoring of the serialization thread.
 */
class PBSerializationThread {
public:
    /**
     * @brief Initialize the serialization thread module
     * 
     * @param ringBuffer Reference to the ring buffer to read samples from
     * @return true if initialization was successful
     */
    static bool initialize(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer);
    
    /**
     * @brief Start the serialization thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the serialization thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the serialization thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the serialized buffer
     * 
     * @return Pointer to the serialized buffer
     */
    static SerializedBuffer* getSerializedBuffer();
    
    /**
     * @brief Get the serialization stats
     * 
     * @return String with serialization stats
     */
    static String getStats();
    
    /**
     * @brief Set the maximum timestamp delta
     * 
     * @param maxDelta Maximum timestamp delta in microseconds
     */
    static void setMaxTimestampDelta(uint16_t maxDelta);

private:
    static PBSerializer* serializer_;
    static SerializedBuffer* serializedBuffer_;
    static int threadId_;
    static volatile bool running_;
    static uint32_t samplesProcessedTotal_;
    
    /**
     * @brief Serialization thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);
};

} // namespace serialization
} // namespace baja