#pragma once

#include "adc/adc_handler.hpp"
#include "util/ring_buffer.hpp"
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include <algorithm>

namespace baja {
namespace adc {

/**
 * @brief ADC function module
 * 
 * Provides initialization, sample processing, and monitoring for the ADC.
 */
namespace functions {

    /**
     * @brief Initialize the ADC module
     * 
     * @param mainBuffer Reference to the main ring buffer for SD storage
     * @param fastBuffer Reference to the fast path buffer for network transmission
     * @param csPin ADC chip select pin
     * @param spiInterface SPI interface to use
     * @param settings ADC settings
     * @return true if initialization was successful
     */
    bool initialize(
        buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& mainBuffer,
        buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer,
        uint8_t csPin,
        SPIClass& spiInterface,
        const ADCSettings& settings = ADCSettings());
    
    /**
     * @brief Start ADC sampling
     * 
     * @return true if successful
     */
    bool start();
    
    /**
     * @brief Stop ADC sampling
     * 
     * @return true if successful
     */
    bool stop();
    
    /**
     * @brief Check if ADC is running
     * 
     * @return true if running
     */
    bool isRunning();
    
    /**
     * @brief Process a single ADC sample - main function called in master loop
     * 
     * @return true if a sample was processed
     */
    bool processSample();
    
    /**
     * @brief Configure ADC channels
     * 
     * @param configs Array of channel configurations
     * @param numChannels Number of channels to configure
     * @return true if successful
     */
    bool configureChannels(const ChannelConfig* configs, size_t numChannels);
    
    /**
     * @brief Get the ADC handler instance
     * 
     * @return Pointer to the ADC handler
     */
    ADC7175Handler* getHandler();
    
    /**
     * @brief Get the fast buffer instance
     * 
     * @return Pointer to the fast buffer
     */
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* getFastBuffer();
    
    /**
     * @brief Get timing statistics for ADC processing
     * 
     * @param avgTime Average processing time in microseconds
     * @param minTime Minimum processing time in microseconds
     * @param maxTime Maximum processing time in microseconds
     * @param sampleCount Total samples processed
     */
    void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint64_t& sampleCount);
    
    /**
     * @brief Reset timing statistics
     */
    void resetTimingStats();
    
    /**
     * @brief Get the sample count
     * 
     * @return Number of samples processed
     */
    uint64_t getSampleCount();
    
    /**
     * @brief Get the active channel
     * 
     * @return Active channel index
     */
    uint8_t getActiveChannel();

} // namespace functions

} // namespace adc
} // namespace baja