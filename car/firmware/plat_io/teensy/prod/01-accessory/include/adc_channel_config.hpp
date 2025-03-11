#pragma once

#include <cstdint>
#include "ad717x.hpp"

namespace baja {
namespace adc {

// DB44 connector analog inputs mapping to AD7175 pins
#define DB44_AIN1 AIN6
#define DB44_AIN2 AIN8
#define DB44_AIN3 AIN10
#define DB44_AIN4 AIN12
#define DB44_AIN5 AIN14
#define DB44_AIN6 AIN7

/**
 * @brief ADC Channel Names
 * 
 * Semantic names for each ADC channel mapped to their physical connections
 */
enum class ChannelName {
    AXLE_TORQUE_FRONT_LEFT = 0,
    AXLE_TORQUE_FRONT_RIGHT,
    AXLE_TORQUE_REAR_LEFT,
    AXLE_TORQUE_REAR_RIGHT,
    
    AXLE_RPM_FRONT_RIGHT,
    AXLE_RPM_FRONT_LEFT,
    AXLE_RPM_REAR,
    
    SHOCK_LEN_FRONT_RIGHT,
    SHOCK_LEN_FRONT_LEFT,
    SHOCK_LEN_REAR_RIGHT,
    SHOCK_LEN_REAR_LEFT,
    
    STEERING_ANGLE,
    TACHOMETER,
    POWER_USE,
    
    BRAKE_PRESSURE_FRONT,
    BRAKE_PRESSURE_REAR,
    
    NUM_CHANNELS  // Always last - gives number of channels
};

/**
 * @brief Channel Configuration Structure
 * 
 * Defines an individual ADC channel configuration
 */
struct ChannelConfig {
    ChannelName name;                    // Semantic name of the channel
    uint8_t channelIndex;                // Physical channel index on the ADC
    bool enabled;                        // Whether this channel is enabled
    bool bipolar;                        // Bipolar/unipolar mode
    ad717x_analog_inputs_t inputPins;    // Input pin configuration
    ad717x_ref_source_t referenceSource; // Reference voltage source
    bool inputBufferEnabled;             // Enable input buffer
    bool referenceBufferEnabled;         // Enable reference buffer
    double gain;                         // Gain value
};

/**
 * @brief ADC Configuration Structure
 * 
 * Overall configuration for the ADC
 */
struct AdcConfig {
    uint8_t samplingRateKhz;                              // Sampling rate in kHz
    ad717x_odr_t outputDataRate;                          // Output data rate register value
    ad717x_order_t filterOrder;                           // Filter order
    ad717x_enhfilt_t enhFilter;                           // Enhanced filter
    bool enhFilterEnabled;                                // Enhanced filter enabled
    bool sinc3Map;                                        // SINC3 map
    ChannelConfig channels[static_cast<int>(ChannelName::NUM_CHANNELS)]; // Channel configurations
};

/**
 * @brief Get the ADC configuration
 * 
 * @return const AdcConfig& Reference to the ADC configuration
 */
const AdcConfig& getAdcConfig();

/**
 * @brief Get a channel configuration by name
 * 
 * @param name Channel name
 * @return const ChannelConfig& Reference to the channel configuration
 */
const ChannelConfig& getChannelConfig(ChannelName name);

/**
 * @brief Get a channel name by index
 * 
 * @param channelIndex Physical channel index
 * @return ChannelName corresponding channel name
 */
ChannelName getChannelNameByIndex(uint8_t channelIndex);

} // namespace adc
} // namespace baja