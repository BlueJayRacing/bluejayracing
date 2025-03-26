/**
 * @file adc_channel_config.hpp
 * @brief ADC channel configuration definitions (header-only)
 */

#pragma once

#include "ad717x.hpp"
#include "util/channel_mapping.hpp"
#include "util/debug_util.hpp"
#include "config/config.hpp"
#include <string>
#include <vector>

namespace baja {
namespace adc {

constexpr int ADC_CHANNEL_COUNT = 16;

/**
 * @brief Configuration for an ADC channel
 */
struct ChannelConfig {
    uint8_t channelIndex;                      // ADC channel index (0-15)
    ad717x_analog_inputs_t analogInputs;       // Analog input configuration
    double gain;                               // Channel gain
    uint8_t setupIndex;                        // Setup configuration index (0-7)
    bool enabled;                              // Whether the channel is enabled
    std::string name;                          // Semantic name of the channel
};

/**
 * @brief Initialize the ADC channel configurations based on default mappings
 * 
 * @param channelConfigs Array of channel configurations to initialize
 * @param enableAllChannels Whether to enable all channels regardless of default settings
 * @return true if initialization was successful, false otherwise
 */
inline bool initializeChannelConfigs(ChannelConfig* channelConfigs, bool enableAllChannels) {
    if (!channelConfigs) {
        util::Debug::error(F("Invalid channel configuration array"));
        return false;
    }
    
    util::Debug::info(F("Initializing channel configurations"));
    
    // Get all channel names
    const auto& allChannelNames = util::getAllChannelNames();
    
    // Initialize each channel configuration
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        auto channelName = allChannelNames[i];
        
        // Set basic properties
        channelConfigs[i].channelIndex = i;
        channelConfigs[i].analogInputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        channelConfigs[i].analogInputs.ainp.neg_input = REF_M; // Use common negative reference
        channelConfigs[i].gain = config::ADC_DEFAULT_GAIN;
        channelConfigs[i].setupIndex = 0;
        
        // Set name based on mapping
        channelConfigs[i].name = util::getChannelNameString(channelName);
        
        // Set enabled status based on channel type or configuration flag
        channelConfigs[i].enabled = enableAllChannels ? true : util::shouldChannelBeEnabled(channelName);
    }
    
    util::Debug::info(F("Channel configurations initialized successfully"));
    return true;
}

/**
 * @brief Print channel configuration information to debug output
 * 
 * @param channelConfigs Array of channel configurations to print
 */
inline void printChannelConfigs(const ChannelConfig* channelConfigs) {
    if (!channelConfigs) {
        util::Debug::error(F("Invalid channel configuration array"));
        return;
    }
    
    util::Debug::info(F("\n===== ADC Channel Configurations ====="));
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        const auto& config = channelConfigs[i];
        util::Debug::info(F("Channel ") + String(i) + 
                         F(": ") + String(config.name.c_str()) + 
                         F(" - ") + String(config.enabled ? "Enabled" : "Disabled") +
                         F(", Gain: ") + String(config.gain));
    }
    util::Debug::info(F("=====================================\n"));
}


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
        odrSetting(SPS_5000) {}  // Set for 50kHz total sampling rate across all channels
};


} // namespace adc
} // namespace baja