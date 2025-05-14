#include "adc_channel_config.hpp"

namespace baja {
namespace adc {

// ADC configuration singleton
static const AdcConfig adcConfig = {
    .samplingRateKhz = 1, // 10kHz sampling rate
    .outputDataRate = SPS_10000, // Use highest possible sample rate
    // .outputDataRate = SPS_250000, // Use highest possible sample rate
    .filterOrder = SINC3, // Using SINC3 for higher speed
    .enhFilter = SPS27_DB47_MS36P7, // Enhanced filter for better rejection
    .enhFilterEnabled = false, // Disabled to maximize speed
    .sinc3Map = true, // Using SINC3 for faster operation
    
    // Channel configurations - using all 16 channels
    .channels = {
        // AXLE_TORQUE_FRONT_LEFT
        {
            .name = ChannelName::AXLE_TORQUE_FRONT_LEFT,
            .channelIndex = 0,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN0, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // AXLE_TORQUE_FRONT_RIGHT
        {
            .name = ChannelName::AXLE_TORQUE_FRONT_RIGHT,
            .channelIndex = 1,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN1, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // AXLE_TORQUE_REAR_LEFT
        {
            .name = ChannelName::AXLE_TORQUE_REAR_LEFT,
            .channelIndex = 2,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN2, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // AXLE_TORQUE_REAR_RIGHT
        {
            .name = ChannelName::AXLE_TORQUE_REAR_RIGHT,
            .channelIndex = 3,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN3, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // AXLE_RPM_FRONT_RIGHT
        {
            .name = ChannelName::AXLE_RPM_FRONT_RIGHT,
            .channelIndex = 4,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN4, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // AXLE_RPM_FRONT_LEFT
        {
            .name = ChannelName::AXLE_RPM_FRONT_LEFT,
            .channelIndex = 5,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN5, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // AXLE_RPM_REAR
        {
            .name = ChannelName::AXLE_RPM_REAR,
            .channelIndex = 6,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN6, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // SHOCK_LEN_FRONT_RIGHT
        {
            .name = ChannelName::SHOCK_LEN_FRONT_RIGHT,
            .channelIndex = 7,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN7, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // SHOCK_LEN_FRONT_LEFT
        {
            .name = ChannelName::SHOCK_LEN_FRONT_LEFT,
            .channelIndex = 8,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN8, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // SHOCK_LEN_REAR_RIGHT
        {
            .name = ChannelName::SHOCK_LEN_REAR_RIGHT,
            .channelIndex = 9,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN9, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // SHOCK_LEN_REAR_LEFT
        {
            .name = ChannelName::SHOCK_LEN_REAR_LEFT,
            .channelIndex = 10,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN10, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // STEERING_ANGLE
        {
            .name = ChannelName::STEERING_ANGLE,
            .channelIndex = 11,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN11, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // TACHOMETER
        {
            .name = ChannelName::TACHOMETER,
            .channelIndex = 12,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN12, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // POWER_USE
        {
            .name = ChannelName::POWER_USE,
            .channelIndex = 13,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN13, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // BRAKE_PRESSURE_FRONT
        {
            .name = ChannelName::BRAKE_PRESSURE_FRONT,
            .channelIndex = 14,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN14, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        },
        
        // BRAKE_PRESSURE_REAR
        {
            .name = ChannelName::BRAKE_PRESSURE_REAR,
            .channelIndex = 15,
            .enabled = true,
            .bipolar = false, // Unipolar mode
            .inputPins = {.ainp = {.pos_input = AIN15, .neg_input = AVDD_AVSS_M}},
            .referenceSource = AVDD_AVSS, // Using AVDD-AVSS as reference
            .inputBufferEnabled = false,
            .referenceBufferEnabled = false,
            .gain = 1.0
        }
    }
};

const AdcConfig& getAdcConfig() {
    return adcConfig;
}

const ChannelConfig& getChannelConfig(ChannelName name) {
    return adcConfig.channels[static_cast<int>(name)];
}

ChannelName getChannelNameByIndex(uint8_t channelIndex) {
    // Search through all channels to find the one with matching index
    for (int i = 0; i < static_cast<int>(ChannelName::NUM_CHANNELS); i++) {
        if (adcConfig.channels[i].channelIndex == channelIndex) {
            return adcConfig.channels[i].name;
        }
    }
    
    // Default to first channel if not found
    return ChannelName::AXLE_TORQUE_FRONT_LEFT;
}

} // namespace adc
} // namespace baja