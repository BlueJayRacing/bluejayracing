#pragma once

#include <cstdint>
#include <atomic>
#include "TeensyThreads.h"
#include "channel_sample.hpp"
#include "adc_channel_config.hpp"

// Forward declare SdFat classes to avoid circular inclusion
class SdFs;
class FsFile;

namespace baja {
namespace data {

/**
 * @brief Multi-segment circular buffer for ADC data
 * 
 * This buffer is divided into multiple segments that can be independently
 * locked and accessed by different threads. It is designed to minimize
 * blocking between the ADC sampling thread and SD card writing thread.
 */
class DataBuffer {
public:
    // Constants for buffer configuration
    static constexpr uint8_t NUM_SEGMENTS = 5;
    static constexpr size_t DEFAULT_BUFFER_SIZE = 480 * 1024; // 480KB total
    static constexpr float SD_WRITE_THRESHOLD = 0.8f; // Trigger SD write at 80% full (4/5 segments)

    /**
     * @brief Buffer segment status
     */
    enum class SegmentStatus {
        EMPTY,          // Available for writing
        FILLING,        // Currently being written to by ADC thread
        FULL,           // Ready for SD card writing
        READING,        // Currently being read by SD card thread
        READY_FOR_MQTT  // Processed and ready for MQTT downsampling
    };

    /**
     * @brief Construct a new Data Buffer
     * 
     * @param size Total buffer size in bytes (default: 480KB)
     */
    explicit DataBuffer(size_t size = DEFAULT_BUFFER_SIZE);
    
    /**
     * @brief Destroy the Data Buffer
     */
    ~DataBuffer();

    /**
     * @brief Push a sample into the buffer
     * 
     * @param sample The sample to push
     * @return true if successful, false if buffer is full
     */
    bool push(const ChannelSample& sample);
    
    /**
     * @brief Push samples in batch for more efficient operation
     * 
     * @param samples Array of samples
     * @param count Number of samples to push
     * @return int Number of samples successfully pushed
     */
    int pushBatch(const ChannelSample* samples, size_t count);

    /**
     * @brief Check if SD writing should be triggered
     * 
     * @return true if SD writing should start
     */
    bool shouldTriggerSDWrite() const;

    /**
     * @brief Begin SD card write operation on the next available full segment
     * 
     * @return Pointer to segment data and its size, or nullptr if no segment is ready
     */
    std::pair<const ChannelSample*, size_t> beginSDWrite();

    /**
     * @brief Complete SD card write operation on the current segment
     * 
     * @param success Whether the write was successful
     */
    void completeSDWrite(bool success);

    /**
     * @brief Get downsampled data for MQTT transmission
     * 
     * @param ratio Downsampling ratio (1 = no downsampling)
     * @param lastSampleIndex Last sample index processed in previous call
     * @param buffer Output buffer to store downsampled data
     * @param maxSamples Maximum number of samples to retrieve
     * @return std::pair<size_t, size_t> Number of samples retrieved and next sample index
     */
    std::pair<size_t, size_t> getDownsampledData(
        uint8_t ratio, 
        size_t lastSampleIndex,
        ChannelSample* buffer, 
        size_t maxSamples
    );

    /**
     * @brief Get the total number of samples in the buffer
     * 
     * @return size_t Number of samples
     */
    size_t size() const;

    /**
     * @brief Check if the buffer is empty
     * 
     * @return true if empty
     */
    bool isEmpty() const;

    /**
     * @brief Check if the buffer is full
     * 
     * @return true if full
     */
    bool isFull() const;

    /**
     * @brief Get the raw buffer capacity in samples
     * 
     * @return size_t Buffer capacity
     */
    size_t capacity() const;

    /**
     * @brief Get the fill percentage of the buffer
     * 
     * @return float Fill percentage (0.0 to 1.0)
     */
    float fillPercentage() const;

    /**
     * @brief Set the downsampling ratio for MQTT
     * 
     * @param ratio New downsampling ratio (1 = no downsampling)
     */
    void setDownsamplingRatio(uint8_t ratio);

    /**
     * @brief Get the current downsampling ratio
     * 
     * @return uint8_t Current ratio
     */
    uint8_t getDownsamplingRatio() const;

private:
    // Buffer segments
    struct Segment {
        ChannelSample* samples;          // Array of samples
        size_t capacity;                 // Capacity in samples
        size_t count;                    // Current sample count
        SegmentStatus status;            // Current status
        Threads::Mutex mutex;            // Mutex for thread-safe access
        
        Segment() : samples(nullptr), capacity(0), count(0), status(SegmentStatus::EMPTY) {}
    };

    // Get next segment index with wrapping
    size_t nextSegmentIndex(size_t current) const;

    // Get segment with specified status, returns -1 if no segment found
    int findSegmentWithStatus(SegmentStatus status) const;

    // Calculate the segment size in samples
    size_t calculateSegmentSize(size_t totalSize) const;

    // Initialize the buffer memory and segment structures
    void initializeBuffer(size_t totalSize);

private:
    // Buffer memory allocation - using regular pointer 
    // (memory will be allocated with extmem_malloc in initialization)
    ChannelSample* buffer_;
    size_t totalCapacity_;           // Total capacity in samples
    size_t segmentCapacity_;         // Capacity per segment in samples
    
    // Segments
    Segment segments_[NUM_SEGMENTS];
    
    // Thread synchronization and status tracking
    Threads::Mutex globalMutex_;     // For operations affecting multiple segments
    volatile size_t currentWriteSegment_;  // Current segment being written to
    volatile size_t currentReadSegment_;   // Current segment being read from
    volatile size_t totalSamples_;         // Total samples in buffer
    volatile bool sdWriteTriggered_;       // Flag to indicate SD write needed
    volatile uint8_t downsamplingRatio_;   // Current downsampling ratio for MQTT
    
    // Prevent copying
    DataBuffer(const DataBuffer&) = delete;
    DataBuffer& operator=(const DataBuffer&) = delete;
};

} // namespace data
} // namespace baja