#pragma once

#include <Arduino.h>
#include <SdFat.h>
#include <string>
#include <vector>
#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_channel_config.hpp"

namespace baja {
namespace storage {

/**
 * @brief SD card writer for data logging
 * 
 * Handles writing data to the SD card in CSV format.
 * Uses SdFat library with SDIO support for optimal performance.
 */
class SDWriter {
public:
    /**
     * @brief Construct a new SD Writer
     * 
     * @param ringBuffer Reference to the ring buffer to read samples from
     */
    SDWriter(buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer);
    
    /**
     * @brief Destroy the SD Writer
     */
    ~SDWriter();
    
    /**
     * @brief Initialize the SD card
     * 
     * @param chipSelect SD card chip select pin (not used for SDIO, but kept for compatibility)
     * @return true if initialization was successful
     */
    bool begin(uint8_t chipSelect = 254);
    
    /**
     * @brief Set channel names for CSV header
     * 
     * @param channelConfigs Vector of channel configurations
     */
    void setChannelNames(const std::vector<adc::ChannelConfig>& channelConfigs);
    
    /**
     * @brief Process data from the ring buffer
     * 
     * This function should be called regularly from the main loop.
     * It reads data from the ring buffer and writes it to the SD card.
     * 
     * @return Number of samples written
     */
    size_t process();
    
    /**
     * @brief Create a new data file
     * 
     * Creates a new file with a timestamp in the filename.
     * 
     * @param addHeader Whether to add a CSV header to the file
     * @return true if file creation was successful
     */
    bool createNewFile(bool addHeader = true);
    
    /**
     * @brief Close the current file
     * 
     * @return true if file was closed successfully
     */
    bool closeFile();
    
    /**
     * @brief Get the current filename
     * 
     * @return Current filename
     */
    std::string getCurrentFilename() const;
    
    /**
     * @brief Get the bytes written since last rotation
     * 
     * @return Bytes written to the current file
     */
    size_t getBytesWritten() const;
    
    /**
     * @brief Check if it's time to rotate the file
     * 
     * @return true if the file should be rotated
     */
    bool shouldRotateFile() const;
    
    /**
     * @brief Flush any buffered data to the card
     * 
     * @return true if successful
     */
    bool flush();

private:
    // Reference to the ring buffer
    buffer::RingBuffer<data::ChannelSample, adc::RING_BUFFER_SIZE>& ringBuffer_;
    
    // SD card objects
    SdFs sd_;
    FsFile dataFile_;
    
    // Channel names
    std::vector<std::string> channelNames_;
    
    // Current filename
    std::string currentFilename_;
    
    // Write buffer position
    size_t writeBufferPos_;
    
    // Tracking for file rotation
    uint32_t fileCreationTime_;
    size_t bytesWritten_;
    
    /**
     * @brief Generate a filename with timestamp
     * 
     * @return Timestamp-based filename
     */
    std::string generateFilename() const;
    
    /**
     * @brief Write CSV header to the file
     * 
     * @return true if successful
     */
    bool writeHeader();
    
    /**
     * @brief Write the buffer to SD card
     * 
     * @return true if successful
     */
    bool flushBuffer();
    
    /**
     * @brief Add a line to the write buffer
     * 
     * @param line Line to add
     * @return true if successful
     */
    bool addLineToBuffer(const std::string& line);
};

} // namespace storage
} // namespace baja