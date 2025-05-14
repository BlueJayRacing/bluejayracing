#pragma once

// Define file format settings
// Set to 1 for CSV output, 0 for binary output
#define SD_WRITER_USE_CSV_FORMAT 1

#include <Arduino.h>
#include <TeensyThreads.h>
#include <SdFat.h>
#include "data_buffer.hpp"

// Forward declaration to avoid circular inclusion
namespace baja {
namespace data {
class DataBuffer;
}
}

namespace baja {
namespace sd {


/**
 * @brief SD Writer Thread
 * 
 * Thread responsible for writing data from the buffer to the SD card
 * when triggered by the data buffer.
 */
class SDWriterThread {
public:
    /**
     * @brief Construct a new SD Writer Thread
     * 
     * @param dataBuffer Reference to the data buffer
     * @param chipSelectPin SD card chip select pin
     * @param sdConfig SD card configuration
     */
    SDWriterThread(data::DataBuffer& dataBuffer, uint8_t chipSelectPin, SdSpiConfig sdConfig);
    
    /**
     * @brief Destroy the SD Writer Thread
     */
    ~SDWriterThread();

    /**
     * @brief Start the SD writer thread
     * 
     * @return int Thread ID or negative value on error
     */
    int start();
    
    /**
     * @brief Stop the SD writer thread
     */
    void stop();
    
    /**
     * @brief Check if the thread is running
     * 
     * @return true if running
     */
    bool isRunning() const;
    
    /**
     * @brief Get the number of bytes written to SD
     * 
     * @return uint64_t Bytes written
     */
    uint64_t getBytesWritten() const;
    
    /**
     * @brief Get the error count
     * 
     * @return uint32_t Error count
     */
    uint32_t getErrorCount() const;
    
    /**
     * @brief Set the base filename
     * 
     * @param basename Base filename (without extension)
     */
    void setBasename(const char* basename);
    
    /**
     * @brief Force the creation of a new file
     */
    void rotateFile();

private:
    /**
     * @brief Main thread function
     */
    static void threadFunction(void* arg);
    
    /**
     * @brief Initialize the SD card
     * 
     * @return true if successful
     */
    bool initializeSD();
    
    /**
     * @brief Create a new data file
     * 
     * @return true if successful
     */
    bool createNewFile();
    
    /**
     * @brief Write a header to the current file
     * 
     * @return true if successful
     */
    bool writeFileHeader();
    
    /**
     * @brief Write a buffer of samples to the SD card
     * 
     * @param samples Pointer to sample buffer
     * @param count Number of samples
     * @return true if successful
     */
    bool writeToSD(const data::ChannelSample* samples, size_t count);

private:
    // Data buffer reference
    data::DataBuffer& dataBuffer_;
    
    // SD card variables
    SdFs sd_;
    FsFile dataFile_;
    uint8_t chipSelectPin_;
    SdSpiConfig sdConfig_;
    
    // Thread control
    int threadId_;
    volatile bool running_;
    
    // File management
    char basename_[64];
    uint32_t fileCounter_;
    
    // Statistics
    volatile uint64_t bytesWritten_;
    volatile uint32_t errorCount_;
    
    // Synchronization
    Threads::Mutex globalMutex_;
    // Prevent copying
    SDWriterThread(const SDWriterThread&) = delete;
    SDWriterThread& operator=(const SDWriterThread&) = delete;
};

} // namespace sd
} // namespace baja