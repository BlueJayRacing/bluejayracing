#pragma once

#include <cstdint>
#include <array>
#include <string>
#include "ad717x.hpp"

namespace baja {
namespace adc {

/**
 * @brief Configuration for each ADC channel
 * 
 * Defines settings for a single ADC channel, including the analog inputs,
 * gain settings, and channel name.
 */
struct ChannelConfig {
    uint8_t channelIndex;                     // Physical ADC channel index (0-15)
    ad717x_analog_inputs_t analogInputs;      // Configured analog inputs
    double gain;                              // Channel gain setting
    uint8_t setupIndex;                       // Which setup to use (0-7)
    std::string name;                         // Human-readable channel name
    bool enabled;                             // Whether channel is enabled
    
    // Default constructor
    ChannelConfig() : 
        channelIndex(0), 
        gain(1.0), 
        setupIndex(0), 
        name("UNNAMED"), 
        enabled(false) {}
    
    // Constructor with all parameters
    ChannelConfig(
        uint8_t index, 
        ad717x_analog_inputs_t inputs, 
        double channelGain, 
        uint8_t setup, 
        const std::string& channelName,
        bool isEnabled = true) : 
            channelIndex(index), 
            analogInputs(inputs), 
            gain(channelGain), 
            setupIndex(setup), 
            name(channelName),
            enabled(isEnabled) {}
};

/**
 * @brief Holds the global ADC settings
 * 
 * Contains overall ADC settings like reference source and operating mode.
 */
struct ADCSettings {
    ad717x_device_type_t deviceType;      // AD7175-8 or other compatible device
    ad717x_ref_source_t referenceSource;  // Internal or external reference
    ad717x_mode_t operatingMode;          // Continuous or single conversion
    bool readStatusWithData;              // Whether to read status with each sample
    ad717x_odr_t odrSetting;                   // Output data rate setting
    
    // Default constructor with sensible defaults
    ADCSettings() : 
        deviceType(ID_AD7175_8),
        referenceSource(INTERNAL_REF),
        operatingMode(CONTINUOUS),
        readStatusWithData(true),
        odrSetting(SPS_50000) {}  // Set for 50kHz total sampling rate across all channels
};

// Number of ADC channels available on AD7175-8
constexpr uint8_t ADC_CHANNEL_COUNT = 16;

// Define the size of each sample in bytes (timestamp + channel + value)
constexpr size_t SAMPLE_SIZE_BYTES = sizeof(uint64_t) + sizeof(uint8_t) + sizeof(uint32_t);

// Memory allocation constants
constexpr size_t RING_BUFFER_SIZE = 24000;  // Size in number of samples
constexpr size_t SD_BUFFER_SIZE = 64 * 1024; // SD write buffer size (64KB)
constexpr size_t SAMPLES_PER_SD_BLOCK = 1500; // How many samples to process at once

} // namespace adc
} // namespace baja

// Declare external buffer arrays to be defined in main.cpp
extern uint8_t sdWriterBuffer[];
extern baja::adc::ChannelConfig channelConfigsArray[];