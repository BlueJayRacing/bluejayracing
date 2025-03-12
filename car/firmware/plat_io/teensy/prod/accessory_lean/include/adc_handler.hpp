#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include "ad717x.hpp"
#include "ring_buffer.hpp"
#include "adc_channel_config.hpp"
#include "sample_data.hpp"

namespace baja {
namespace adc {

/**
 * @brief Handler for the AD7175-8 ADC
 * 
 * Manages the AD7175-8 ADC, handling initialization, configuration,
 * and flag-based sampling (for main loop processing).
 */
class ADC7175Handler {
public:
    /**
     * @brief Construct a new ADC7175Handler
     * 
     * @param ringBuffer Reference to the ring buffer to store samples
     */
    ADC7175Handler(buffer::RingBuffer<data::ChannelSample, RING_BUFFER_SIZE>& ringBuffer);
    
    /**
     * @brief Destroy the ADC7175Handler
     */
    ~ADC7175Handler();
    
    /**
     * @brief Initialize the ADC
     * 
     * Sets up the SPI interface, configures the ADC, and sets up the interrupt.
     * 
     * @param csPin Chip select pin for the ADC
     * @param drdyPin Data ready pin (for interrupt)
     * @param spiInterface SPI interface to use
     * @param settings ADC settings
     * @return true if initialization was successful
     */
    bool begin(uint8_t csPin, uint8_t drdyPin, SPIClass& spiInterface, 
               const ADCSettings& settings = ADCSettings());
    
    /**
     * @brief Configure multiple channels
     * 
     * @param configs Array of channel configurations
     * @param numChannels Number of channels to configure
     * @return true if configuration was successful
     */
    bool configureChannels(const ChannelConfig* configs, size_t numChannels);
    
    /**
     * @brief Configure a single channel
     * 
     * @param config Channel configuration
     * @return true if configuration was successful
     */
    bool configureChannel(const ChannelConfig& config);
    
    /**
     * @brief Start continuous sampling
     * 
     * Enables the ADC interrupt and starts continuous conversion mode.
     * 
     * @return true if successful
     */
    bool startSampling();
    
    /**
     * @brief Stop sampling
     * 
     * Disables the ADC interrupt and puts the ADC in standby mode.
     * 
     * @return true if successful
     */
    bool stopSampling();
    
    /**
     * @brief Process ADC data when available
     * 
     * This function should be called regularly from the main loop.
     * It checks if new data is available and processes it if so.
     * 
     * @return true if a sample was processed
     */
    bool processData();
    
    /**
     * @brief Read a single sample from the ADC
     * 
     * Reads the current sample and returns the data.
     * 
     * @param sample Output parameter for the sample data
     * @return true if read was successful
     */
    bool readSample(ad717x_data_t& sample);
    
    /**
     * @brief Get the active channel index
     * 
     * @return Currently active channel index
     */
    uint8_t getActiveChannel() const;
    
    /**
     * @brief Set the interrupt priority
     * 
     * @param priority Priority level (0 = highest, 255 = lowest)
     */
    void setInterruptPriority(uint8_t priority);
    
    /**
     * @brief Get a copy of all channel configurations
     * 
     * @return Vector of channel configurations
     */
    std::vector<ChannelConfig> getChannelConfigs() const;
    
    /**
     * @brief Get the sample count
     * 
     * @return Number of samples collected
     */
    uint64_t getSampleCount() const;
    
    /**
     * @brief Reset the sample count
     */
    void resetSampleCount();
    
    /**
     * @brief Get data ready flag status
     * 
     * @return true if data is ready to be read
     */
    bool isDataReady() const;
    
    /**
     * @brief Signal that data is ready (called by ISR)
     * 
     * This is called by the ISR to set the dataReady flag.
     */
    void signalDataReady();
    
    /**
     * @brief Reset ADC with manual SPI sequence
     * 
     * Performs a manual reset sequence by sending 0xFF bytes
     */
    void resetADC();
    
    /**
     * @brief Set up static interrupt handler
     * 
     * Sets up a static interrupt handler that calls the instance method.
     * This is needed because ISRs must be static functions.
     * 
     * @param instance Pointer to the ADC7175Handler instance
     */
    static void setInterruptHandler(ADC7175Handler* instance);

private:
    // Reference to the ring buffer
    buffer::RingBuffer<data::ChannelSample, RING_BUFFER_SIZE>& ringBuffer_;
    
    // AD7175-8 driver
    AD717X adcDriver_;
    
    // Channel configurations stored in RAM2
    ChannelConfig* channelConfigs_;
    
    // Pins and interface
    uint8_t csPin_;
    uint8_t drdyPin_;
    SPIClass* spiInterface_;
    
    // Active channel
    volatile uint8_t activeChannel_;
    
    // Sample counter
    volatile uint64_t sampleCount_;
    
    // Sampling state
    volatile bool samplingActive_;
    
    // Data ready flag (set by ISR, checked by main loop)
    volatile bool dataReady_;
    
    // Static pointer to the handler instance for ISR
    static ADC7175Handler* instance_;
    
    // Static ISR that only sets the flag
    static void isr();
};

} // namespace adc
} // namespace baja