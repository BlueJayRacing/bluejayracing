#include "adc_handler.hpp"

namespace baja {
namespace adc {

// Define error code constants from AD717X.cpp
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR    -2 /* Communication error on receive */
#define TIMEOUT     -3 /* A timeout has occured */

ADC7175Handler::ADC7175Handler(buffer::RingBuffer<data::ChannelSample, baja::config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      csPin_(0),
      spiInterface_(nullptr),
      activeChannel_(0),
      sampleCount_(0),
      samplingActive_(false) {
    channelConfigs_ = nullptr;
}

ADC7175Handler::~ADC7175Handler() {
    stopSampling();
}

bool ADC7175Handler::begin(uint8_t csPin, SPIClass& spiInterface, 
                          const ADCSettings& settings) {
    util::Debug::info("ADC: Initializing");
    csPin_ = csPin;
    spiInterface_ = &spiInterface;
    
    // Configure the CS pin as output
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH); // Deselect ADC
    
    // Create the initialization parameters
    ad717x_init_param_t initParam;
    initParam.active_device = settings.deviceType;
    initParam.mode = settings.operatingMode;
    initParam.stat_on_read_en = settings.readStatusWithData;
    initParam.ref_en = (settings.referenceSource == INTERNAL_REF);
    
    // Set up the channel map initially with all channels disabled
    initParam.chan_map.resize(ADC_CHANNEL_COUNT);
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        initParam.chan_map[i].channel_enable = (i < 2); // Only enable channels 0 and 1 for initial test
        initParam.chan_map[i].setup_sel = 0;
        
        // Default to AINx for positive input and REF_M for negative
        initParam.chan_map[i].inputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i);
        initParam.chan_map[i].inputs.ainp.neg_input = REF_M;
    }
    
    // Set up one default setup
    ad717x_setup_t setup;
    setup.setup.bi_polar = false;
    setup.setup.input_buff = true;
    setup.setup.ref_buff = true;
    setup.setup.ref_source = settings.referenceSource;
    setup.filter_config.odr = settings.odrSetting;
    setup.gain = 1.0;
    
    // Add the setup to the parameters
    initParam.setups.push_back(setup);
    
    // Initialize the ADC with debugging
    int result = adcDriver_.init(initParam, spiInterface_, csPin_, SPISettings(10000000, MSBFIRST, SPI_MODE3));
    if (result < 0) {
        util::Debug::error("ADC: Initialization failed with code " + String(result));
        return false;
    }
    
    // Read the ID register to verify
    result = adcDriver_.readRegister(AD717X_ID_REG);
    if (result >= 0) {
        ad717x_st_reg_t* idReg = adcDriver_.getReg(AD717X_ID_REG);
        if (idReg) {
            uint16_t chipId = idReg->value & AD717X_ID_REG_MASK;
            if (chipId == AD7175_8_ID_REG_VALUE) {
                util::Debug::info("ADC: Confirmed device is AD7175-8");
            } else {
                util::Debug::warning("ADC: Unexpected chip ID: 0x" + String(chipId, HEX));
            }
        }
    }
    
    util::Debug::info("ADC: Initialization complete");
    return true;
}

bool ADC7175Handler::configureChannels(const ChannelConfig* configs, size_t numChannels) {
    util::Debug::info("ADC: Configuring channels");
    
    // Store pointer to configurations
    channelConfigs_ = const_cast<ChannelConfig*>(configs);
    
    // Configure each channel
    for (size_t i = 0; i < numChannels; i++) {
        util::Debug::detail("ADC: Configuring channel " + String(configs[i].channelIndex) + 
                         " (" + String(configs[i].name.c_str()) + ")");
        
        if (!configureChannel(configs[i])) {
            util::Debug::error("ADC: Failed to configure channel " + String(i));
            return false;
        }
    }
    
    util::Debug::info("ADC: All channels configured successfully");
    return true;
}

bool ADC7175Handler::configureChannel(const ChannelConfig& config) {
    // Set the channel status (enabled/disabled)
    int result = adcDriver_.setChannelStatus(config.channelIndex, config.enabled);
    if (result < 0) {
        util::Debug::error("ADC: setChannelStatus failed with code " + String(result));
        return false;
    }
    
    // Connect the analog inputs
    result = adcDriver_.connectAnalogInput(config.channelIndex, config.analogInputs);
    if (result < 0) {
        util::Debug::error("ADC: connectAnalogInput failed with code " + String(result));
        return false;
    }
    
    // Assign the setup
    result = adcDriver_.assignSetup(config.channelIndex, config.setupIndex);
    if (result < 0) {
        util::Debug::error("ADC: assignSetup failed with code " + String(result));
        return false;
    }
    
    // Set the gain
    result = adcDriver_.setGain(config.gain, config.setupIndex);
    if (result < 0) {
        util::Debug::error("ADC: setGain failed with code " + String(result));
        return false;
    }
    
    return true;
}

bool ADC7175Handler::startSampling() {
    // Check if we're already sampling
    util::Debug::info("ADC: Starting continuous sampling");
    if (samplingActive_) {
        util::Debug::info("ADC: Sampling already active");
        return true;
    }
    
    // Reset sample count
    sampleCount_ = 0;
    
    // Set the ADC to continuous conversion mode
    int result = adcDriver_.setADCMode(CONTINUOUS);
    if (result < 0) {
        util::Debug::error("ADC: setADCMode failed with code " + String(result));
        return false;
    }
    
    // Mark as started
    samplingActive_ = true;
    
    util::Debug::info("ADC: Sampling started successfully");
    return true;
}

bool ADC7175Handler::stopSampling() {
    // Check if we're already stopped
    if (!samplingActive_) {
        return true;
    }
    
    // Set the ADC to standby mode
    int result = adcDriver_.setADCMode(STANDBY);
    if (result < 0) {
        util::Debug::error("ADC: Failed to stop sampling, code " + String(result));
        return false;
    }
    
    // Mark as stopped
    samplingActive_ = false;
    
    util::Debug::info("ADC: Sampling stopped");
    return true;
}

bool ADC7175Handler::pollForSample(uint32_t timeout_ms) {
    // Check if sampling is active
    if (!samplingActive_) {
        return false;
    }
    
    // Wait for ready with timeout
    uint32_t timeout = timeout_ms == 0 ? 0xFFFFFFFF : timeout_ms * 10; // Convert to internal units
    int result = adcDriver_.waitForReady(timeout);
    
    if (result < 0) {
        if (result == TIMEOUT) {
            // This is normal if timeout_ms was specified
            return false;
        }
        
        util::Debug::warning("ADC: waitForReady failed with code " + String(result));
        return false;
    }
    
    // Read the sample
    ad717x_data_t sample;
    if (!readSample(sample)) {
        return false;
    }
    
    // Create a channel sample and add to ring buffer
    data::ChannelSample channelSample(
        micros(),                   // Microsecond timestamp
        sample.status.active_channel, // Channel index
        sample.value                // Raw ADC value
    );
    
    // Add to the ring buffer
    if (!ringBuffer_.write(channelSample)) {
        util::Debug::warning("ADC: Ring buffer full, sample dropped");
        return false;
    }
    
    // Update counters and active channel
    sampleCount_++;
    activeChannel_ = sample.status.active_channel;
    
    // Debug every 1,000,000 samples
    if (sampleCount_ % 1000000 == 0) {
        util::Debug::info("ADC: Sample #" + String(sampleCount_) + 
                       ": Channel=" + String(sample.status.active_channel) + 
                       ", Value=" + String(sample.value));
    }
    
    return true;
}

bool ADC7175Handler::readSample(ad717x_data_t& sample) {
    // Read a sample from the ADC
    int result = adcDriver_.contConvReadData(&sample);
    
    if (result < 0) {
        util::Debug::warning("ADC: Error reading data, code " + String(result));
        return false;
    }
    
    return true;
}

uint8_t ADC7175Handler::getActiveChannel() const {
    return activeChannel_;
}

std::vector<ChannelConfig> ADC7175Handler::getChannelConfigs() const {
    // Create a temporary vector for the result
    std::vector<ChannelConfig> configs;
    
    // Check if channelConfigs_ is initialized
    if (channelConfigs_ == nullptr) {
        util::Debug::warning("ADC: channelConfigs_ is NULL in getChannelConfigs()");
        return configs;
    }
    
    // Copy channel configurations from the external array
    for (size_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
        if (channelConfigs_[i].enabled) {
            configs.push_back(channelConfigs_[i]);
        }
    }
    
    return configs;
}

uint64_t ADC7175Handler::getSampleCount() const {
    return sampleCount_;
}

void ADC7175Handler::resetSampleCount() {
    sampleCount_ = 0;
}

void ADC7175Handler::resetADC() {
    // Manually reset the ADC by toggling CS and sending 0xFF bytes
    digitalWrite(csPin_, LOW);
    
    // Send 8 bytes of 0xFF for reset
    for (int i = 0; i < 8; i++) {
        spiInterface_->transfer(0xFF);
    }
    
    digitalWrite(csPin_, HIGH);
    
    // Wait for ADC to reset
    delay(10);
    
    util::Debug::info("ADC: Manual reset performed");
}

} // namespace adc
} // namespace baja