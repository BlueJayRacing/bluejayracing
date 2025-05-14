#pragma once

#include <TeensyThreads.h>
#include "adc/adc_handler.hpp"
#include "util/ring_buffer.hpp"
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include <algorithm>

namespace baja {
namespace adc {

/**
 * @brief ADC thread module
 * 
 * Handles initialization, running, and monitoring of the ADC sampling thread.
 */
class ADCThread {
public:
    /**
     * @brief Initialize the ADC thread module
     * 
     * @param ringBuffer Reference to the main ring buffer for SD storage
     * @param fastBuffer Reference to the fast path buffer for network transmission
     * @param csPin ADC chip select pin
     * @param spiInterface SPI interface to use
     * @param settings ADC settings
     * @return true if initialization was successful
     */
    static bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
        buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer,
        uint8_t csPin,
        SPIClass& spiInterface,
        const ADCSettings& settings = ADCSettings());
    
    /**
     * @brief Start the ADC thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the ADC thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the ADC thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the ADC handler instance
     * 
     * @return Pointer to the ADC handler
     */
    static ADC7175Handler* getHandler();
    
    /**
     * @brief Get the fast buffer instance
     * 
     * @return Pointer to the fast buffer
     */
    static buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* getFastBuffer();
    
    /**
     * @brief Configure ADC channels
     * 
     * @param configs Array of channel configurations
     * @param numChannels Number of channels to configure
     * @return true if successful
     */
    static bool configureChannels(const ChannelConfig* configs, size_t numChannels);
    
    /**
     * @brief Process a single ADC sample (for use in main thread)
     * 
     * @return true if a sample was processed
     */
    static bool processSample();

private:
    static ADC7175Handler* adcHandler_;
    static int threadId_;
    static volatile bool running_;
    static uint64_t sampleCount_;
    
    // Fast path buffer for network transmission
    static buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* fastBuffer_;
    
    // Per-channel downsampling counters
    static uint16_t channelSampleCounters_[ADC_CHANNEL_COUNT];
    
    /**
     * @brief ADC sampling thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);

};



} // namespace adc
} // namespace baja