#pragma once

#include <Arduino.h>
#include <SdFat.h>
#include <string>
#include <vector>
#include "ring_buffer.hpp"
#include "sample_data.hpp"
#include "adc_channel_config.hpp"

// Forward declaration of global buffer to fix linkage issues
extern uint8_t sdWriterBuffer[];

namespace baja {
namespace storage {

// Define buffer size constants in bytes
constexpr size_t MIN_WRITE_SIZE = 4 * 1024;      // 4KB minimum write for efficiency
constexpr size_t MAX_BUFFER_SIZE = 256 * 1024;    // 64KB maximum buffer size
constexpr size_t BUFFER_FLUSH_THRESHOLD = 32 * 1024;  // 32KB flush threshold

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
     * This function should be called regularly from the SD writer thread.
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
    
    /**
     * @brief Get the last error code
     * 
     * @return Last error code (0 = no error)
     */
    int getLastError() const;
    
    /**
     * @brief Check if the SD writer is healthy
     * 
     * @return true if no critical errors have occurred
     */
    bool isHealthy() const;
    
    /**
     * @brief Reset the health status
     */
    void resetHealth();

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
    
    // Error tracking
    int lastError_;
    bool healthy_;
    uint32_t lastErrorTime_;
    uint32_t consecutiveErrors_;
    uint32_t totalErrors_;
    uint32_t lastSuccessfulWrite_;
    
    // Performance tracking
    uint32_t lastFlushTime_;
    uint32_t totalFlushes_;
    uint32_t maxFlushTime_;
    
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
    
    /**
     * @brief Record an error
     * 
     * @param errorCode Error code
     * @param errorMessage Error message
     */
    void recordError(int errorCode, const char* errorMessage);
};

} // namespace storage
} // namespace baja