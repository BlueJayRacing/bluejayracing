#pragma once

#include <TeensyThreads.h>
#include "storage/sd_writer.hpp"
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include "adc/adc_channel_config.hpp"

namespace baja {
namespace storage {

/**
 * @brief SD writer thread module
 * 
 * Handles initialization, running, and monitoring of the SD writer thread.
 */
class SDThread {
public:
    /**
     * @brief Initialize the SD thread module
     * 
     * @param ringBuffer Reference to the ring buffer to read samples from
     * @param sdRingBuf Pointer to the SdFat RingBuf buffer
     * @param chipSelect SD card chip select pin (not used for SDIO)
     * @return true if initialization was successful
     */
    static bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
        RingBuf<FsFile, config::SD_RING_BUF_CAPACITY>* sdRingBuf,
        uint8_t chipSelect = 254);
    
    /**
     * @brief Start the SD writer thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the SD writer thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the SD writer thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the SD writer instance
     * 
     * @return Pointer to the SD writer
     */
    static SDWriter* getWriter();
    
    /**
     * @brief Set channel configurations for the SD writer
     * 
     * @param channelConfigs Vector of channel configurations
     */
    static void setChannelConfigs(const std::vector<adc::ChannelConfig>& channelConfigs);
    
    /**
     * @brief Flush any buffered data to the SD card
     * 
     * @return true if successful
     */
    static bool flush();

private:
    static SDWriter* sdWriter_;
    static int threadId_;
    static volatile bool running_;
    static uint32_t totalWritten_;
    
    /**
     * @brief SD writer thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);
};

} // namespace storage
} // namespace baja