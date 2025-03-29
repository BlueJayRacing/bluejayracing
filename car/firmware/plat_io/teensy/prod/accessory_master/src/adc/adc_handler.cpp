#include "adc/adc_handler.hpp"

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

// bool ADC7175Handler::pollForSample(uint32_t timeout_ms) {
//     // Check if sampling is active
//     if (!samplingActive_) {
//         return false;
//     }

//     static int counter = 0;
//     counter ++;
//     uint32_t start_time = micros();
    
//     // Wait for ready with timeout
//     uint32_t timeout = timeout_ms == 0 ? 0xFFFFFFFF : timeout_ms * 10; // Convert to internal units
//     int result = adcDriver_.waitForReady(timeout);
//     if (counter > 40000)Serial.println("It took " + String(micros() - start_time) + "us to wait for ready");


//     if (result < 0) {
//         if (result == TIMEOUT) {
//             // This is normal if timeout_ms was specified
//             return false;
//         }
        
//         util::Debug::warning("ADC: waitForReady failed with code " + String(result));
//         return false;
//     }
    
    


//     start_time = micros();
//     // Read the sample
//     ad717x_data_t sample;
//     if (!readSample(sample)) {
//         return false;
//     }
//     if (counter > 40000)Serial.println("It took " + String(micros() - start_time) + "us to read the sample");
    
//     // Create a channel sample and add to ring buffer
//     data::ChannelSample channelSample(
//         micros(),                   // Microsecond timestamp
//         sample.status.active_channel, // Channel index
//         sample.value                // Raw ADC value
//     );
    
//     start_time = micros();
//     // Add to the ring buffer
//     if (!ringBuffer_.write(channelSample)) {
//         // util::Debug::warning("ADC: Ring buffer full, sample dropped");
//         return false;
//     }
//     if (counter > 40000) Serial.println("It took " + String(micros() - start_time) + "us to write the sample");
//     if (counter > 40000) counter = 0;
    
//     // Update counters and active channel
//     sampleCount_++;
//     activeChannel_ = sample.status.active_channel;
    
//     // Debug every 1,000,000 samples
//     if (sampleCount_ % 1000000 == 0) {
//         util::Debug::info("ADC: Sample #" + String(sampleCount_) + 
//                        ": Channel=" + String(sample.status.active_channel) + 
//                        ", Value=" + String(sample.value));
//     }
    
//     return true;
// }

int ADC7175Handler::pollForSample(uint32_t timeout_ms) {
    // Check if sampling is active
    if (!samplingActive_) {
        return INVALID_VAL;
    }

    // Timing statistics - only log occasionally
    static uint32_t sampleCounter = 0;
    static uint32_t totalWaitTime = 0;
    static uint32_t totalReadTime = 0;
    static uint32_t totalWriteTime = 0;
    static uint32_t samplesSinceLastLog = 0;
    static uint32_t lastLogTime = 0;
    const uint32_t LOG_INTERVAL = 10000; // ms
    
    sampleCounter++;
    samplesSinceLastLog++;
    
    // Start timing for wait operation
    uint32_t wait_start = micros();
    
    // Wait for ready with timeout
    uint32_t timeout = timeout_ms == 0 ? 0xFFFFFFFF : timeout_ms * 10; // Convert to internal units
    int result = adcDriver_.waitForReady(timeout);
    
    // Calculate wait time
    uint32_t wait_time = micros() - wait_start;
    totalWaitTime += wait_time;
    
    // Check wait result
    if (result < 0) {
        if (result == TIMEOUT) {
            // This is normal if timeout_ms was specified
            return AH_TIMEOUT;
        }
        
        // Only log occasionally to avoid spamming
        static uint32_t lastWarnTime = 0;
        uint32_t currentTime = millis();
        if (currentTime - lastWarnTime > 1000) { // Only warn once per second
            util::Debug::warning("ADC: waitForReady failed with code " + String(result));
            lastWarnTime = currentTime;
        }
        return AH_COMM_ERR;
    }
    
    // Start timing for read operation
    uint32_t read_start = micros();
    
    // Read the sample
    ad717x_data_t sample;
    if (!readSample(sample)) {
        return AH_COMM_ERR;
    }

    // Cache the conversion result
    lastConversion_ = sample;
    
    // Calculate read time
    uint32_t read_time = micros() - read_start;
    totalReadTime += read_time;
    
    // Create a channel sample and add to ring buffer
    uint8_t internalChannelId = static_cast<uint8_t>(
        util::mapADCToInternalID(sample.status.active_channel));
    
    data::ChannelSample channelSample(
        micros(),                   // Microsecond timestamp
        internalChannelId,          // Internal channel ID
        sample.value,               // Raw ADC value
        millis()                    // Add recorded time
    );
    
    // Start timing for write operation
    uint32_t write_start = micros();
    
    // Add to the ring buffer
    if (!ringBuffer_.write(channelSample)) {
        // Only log occasionally to avoid spamming
        static uint32_t lastRingBufferWarnTime = 0;
        uint32_t currentTime = millis();
        if (currentTime - lastRingBufferWarnTime > 5000) { // Only warn every 5 seconds
            util::Debug::warning("ADC: Ring buffer full, sample dropped");
            lastRingBufferWarnTime = currentTime;
        }
        return AH_BUFFERBAD;
    }
    
    // Calculate write time
    uint32_t write_time = micros() - write_start;
    totalWriteTime += write_time;
    
    // Log timing statistics every LOG_INTERVAL samples or 10 seconds
    uint32_t currentTime = millis();
    if ((sampleCounter % 40000 == 0) || (currentTime - lastLogTime > LOG_INTERVAL && samplesSinceLastLog > 0)) {
        float avg_wait = (float)totalWaitTime / samplesSinceLastLog;
        float avg_read = (float)totalReadTime / samplesSinceLastLog;
        float avg_write = (float)totalWriteTime / samplesSinceLastLog;
        float avg_total = avg_wait + avg_read + avg_write;
        float samples_per_sec = samplesSinceLastLog * 1000.0f / (currentTime - lastLogTime);
        
        // Check for long waits that could indicate performance issues
        if (wait_time > 100) {
            util::Debug::detail("ADC Long Wait: " + String(wait_time) + "µs for sample #" + String(sampleCounter));
        }
        
        // Only log the first few and then periodically to avoid spam
        if (sampleCounter < 1000 || sampleCounter % 500000 == 0) {
            util::Debug::detail("ADC Poll Timing: " + 
                        String(samplesSinceLastLog) + " samples @ " + 
                        String(samples_per_sec, 1) + " sps, avg=" + 
                        String(avg_total, 1) + "µs (wait=" + 
                        String(avg_wait, 1) + "µs, read=" + 
                        String(avg_read, 1) + "µs, write=" + 
                        String(avg_write, 1) + "µs)");
        }
        
        // Reset timing statistics
        totalWaitTime = 0;
        totalReadTime = 0;
        totalWriteTime = 0;
        samplesSinceLastLog = 0;
        lastLogTime = currentTime;
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
    
    return AH_OK;
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