#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include "ad717x.hpp"
#include "util/ring_buffer.hpp"
#include "adc_channel_config.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace adc {


#define AH_BUFFERBAD     1 /* No error */
#define AH_OK            0 /* No error */
#define AH_INVALID_VAL  -1 /* Invalid argument */
#define AH_COMM_ERR     -2 /* Communication error on receive */
#define AH_TIMEOUT      -3 /* A timeout has occured */
#define AH_DISABLED     -4 /* Channel is disabled */

/**
 * @brief Handler for the AD7175-8 ADC
 * 
 * Manages the AD7175-8 ADC using waitForReady polling approach
 */
class ADC7175Handler {
public:
    /**
     * @brief Construct a new ADC7175Handler
     * 
     * @param ringBuffer Reference to the ring buffer to store samples
     */
    ADC7175Handler(buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer);
    
    /**
     * @brief Destroy the ADC7175Handler
     */
    ~ADC7175Handler();
    
    /**
     * @brief Initialize the ADC
     * 
     * @param csPin Chip select pin for the ADC
     * @param spiInterface SPI interface to use
     * @param settings ADC settings
     * @return true if initialization was successful
     */
    bool begin(uint8_t csPin, SPIClass& spiInterface, 
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
     * @return true if successful
     */
    bool startSampling();
    
    /**
     * @brief Stop sampling
     * 
     * @return true if successful
     */
    bool stopSampling();
    
    /**
     * @brief Poll for and process new ADC data
     * 
     * @param timeout_ms Maximum time to wait for data in milliseconds (0 = infinite)
     * @return true if a sample was processed
     */
    int pollForSample(uint32_t timeout_ms = 0);
    
    /**
     * @brief Get the active channel index
     * 
     * @return Currently active channel index
     */
    uint8_t getActiveChannel() const;
    
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
     * @brief Reset ADC with manual SPI sequence
     */
    void resetADC();

    

    bool getLatestConversion(ad717x_data_t& sample) const {
        sample = lastConversion_;
        return true;
    }

    bool getLastConversionTime(uint64_t& time) const {
        time = lastConversionTime_;
        return true;
    }

private:
    buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer_;
    AD717X adcDriver_;
    ChannelConfig* channelConfigs_;
    uint8_t csPin_;
    SPIClass* spiInterface_;
    volatile uint8_t activeChannel_;
    volatile uint64_t sampleCount_;
    volatile bool samplingActive_;
    ad717x_data_t lastConversion_;
    uint64_t lastConversionTime_;
    
    /**
     * @brief Read a sample from the ADC
     * 
     * @param sample Output parameter for the sample data
     * @return true if read was successful
     */
    bool readSample(ad717x_data_t& sample);
};

} // namespace adc
} // namespace baja