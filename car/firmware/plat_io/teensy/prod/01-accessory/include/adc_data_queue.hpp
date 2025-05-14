#pragma once

#include <cstdint>
#include <array>
#include "TeensyThreads.h"
#include "adc_channel_config.hpp"

namespace baja {
namespace adc {

/**
 * @brief ADC Sample Structure
 * 
 * Contains a single ADC sample with timestamp and channel information
 */
struct AdcSample {
    uint64_t timestamp;      // Microsecond timestamp
    ChannelName channelName; // Channel that was sampled
    uint32_t rawValue;       // Raw ADC value
    
    // Add constructor for easy initialization
    AdcSample(uint64_t ts = 0, ChannelName ch = ChannelName::AXLE_TORQUE_FRONT_LEFT, uint32_t val = 0)
        : timestamp(ts), channelName(ch), rawValue(val) {}
};

/**
 * @brief ADC Data Queue Class
 * 
 * Thread-safe ring buffer for ADC samples stored in TCM memory for fast access.
 * This queue passes data from the ADC thread to processing threads.
 */
class AdcDataQueue {
public:
    /**
     * @brief Construct a new ADC Data Queue
     * 
     * @param bufferSize Size of the ring buffer in number of samples
     */
    explicit AdcDataQueue(size_t bufferSize);
    
    /**
     * @brief Destroy the ADC Data Queue
     */
    ~AdcDataQueue();
    
    /**
     * @brief Push a new sample into the queue
     * 
     * @param sample The sample to push
     * @return true if successful, false if queue is full
     */
    bool push(const AdcSample& sample);
    
    /**
     * @brief Push samples in batch
     * 
     * @param samples Array of samples
     * @param count Number of samples to push
     * @return int Number of samples successfully pushed
     */
    int pushBatch(const AdcSample* samples, size_t count);
    
    /**
     * @brief Pop a sample from the queue
     * 
     * @param sample Reference to store the popped sample
     * @return true if successful, false if queue is empty
     */
    bool pop(AdcSample& sample);
    
    /**
     * @brief Pop multiple samples from the queue
     * 
     * @param samples Array to store the popped samples
     * @param maxCount Maximum number of samples to pop
     * @return int Number of samples actually popped
     */
    int popBatch(AdcSample* samples, size_t maxCount);
    
    /**
     * @brief Get the number of samples in the queue
     * 
     * @return size_t Number of samples
     */
    size_t size() const;
    
    /**
     * @brief Check if the queue is empty
     * 
     * @return true if empty, false otherwise
     */
    bool isEmpty() const;
    
    /**
     * @brief Check if the queue is full
     * 
     * @return true if full, false otherwise
     */
    bool isFull() const;
    
    /**
     * @brief Clear the queue
     */
    void clear();

private:
    // Buffer allocated in TCM for fast access
    AdcSample* buffer_;
    size_t bufferSize_;
    
    // Read and write pointers
    volatile size_t readIndex_;
    volatile size_t writeIndex_;
    volatile size_t count_;
    
    // Thread synchronization
    Threads::Mutex mutex_;
    
    // Prevent copying
    AdcDataQueue(const AdcDataQueue&) = delete;
    AdcDataQueue& operator=(const AdcDataQueue&) = delete;
};

} // namespace adc
} // namespace baja