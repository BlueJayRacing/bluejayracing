#pragma once

#include "storage/sd_writer.hpp"
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include "adc/adc_channel_config.hpp"
#include <vector>

namespace baja {
namespace storage {

/**
 * @brief SD writer function module
 * 
 * Provides initialization, file management, and data writing for SD card storage.
 */
namespace functions {

    /**
     * @brief Initialize the SD module
     * 
     * @param ringBuffer Reference to the ring buffer to read samples from
     * @param sdRingBuf Pointer to the SdFat RingBuf buffer
     * @param chipSelect SD card chip select pin (not used for SDIO)
     * @return true if initialization was successful
     */
    bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
        RingBuf<FsFile, config::SD_RING_BUF_CAPACITY>* sdRingBuf,
        uint8_t chipSelect = 254);
    
    /**
     * @brief Start SD writer
     * 
     * @return true if successful
     */
    bool start();
    
    /**
     * @brief Stop SD writer
     * 
     * @return true if successful
     */
    bool stop();
    
    /**
     * @brief Check if SD writer is running
     * 
     * @return true if running
     */
    bool isRunning();
    
    /**
     * @brief Process and write samples to SD card - called from master loop
     * 
     * Checks if there are enough samples (>= 15) before attempting to write.
     * 
     * @return Number of samples written (0 if no writing occurred)
     */
    size_t process();
    
    /**
     * @brief Get the SD writer instance
     * 
     * @return Pointer to the SD writer
     */
    SDWriter* getWriter();
    
    /**
     * @brief Set channel configurations for the SD writer
     * 
     * @param channelConfigs Vector of channel configurations
     */
    void setChannelConfigs(const std::vector<adc::ChannelConfig>& channelConfigs);
    
    /**
     * @brief Set a custom name for a channel
     * 
     * @param internalChannelId Internal channel ID (0-29)
     * @param customName Custom name for the channel
     */
    void setCustomChannelName(uint8_t internalChannelId, const std::string& customName);

    /**
     * @brief Create a new file for writing
     * 
     * @param addHeader Whether to add a header to the file
     * @return true if successful
     */
    bool createNewFile(bool addHeader = true);
    
    /**
     * @brief Close the current file
     * 
     * @return true if successful
     */
    bool closeFile();
    
    /**
     * @brief Flush any buffered data to the SD card
     * 
     * @return true if successful
     */
    bool flush();
    
    /**
     * @brief Get the current filename
     * 
     * @return Current filename
     */
    std::string getCurrentFilename();
    
    /**
     * @brief Get bytes written to current file
     * 
     * @return Bytes written
     */
    size_t getBytesWritten();
    
    /**
     * @brief Get samples written to current file
     * 
     * @return Samples written
     */
    uint64_t getSamplesWritten();
    
    /**
     * @brief Get timing statistics for SD processing
     * 
     * @param avgTime Average processing time in microseconds
     * @param minTime Minimum processing time in microseconds
     * @param maxTime Maximum processing time in microseconds
     * @param totalWrites Total number of successful write operations
     */
    void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint32_t& totalWrites);
    
    /**
     * @brief Reset timing statistics
     */
    void resetTimingStats();

} // namespace functions

} // namespace storage
} // namespace baja