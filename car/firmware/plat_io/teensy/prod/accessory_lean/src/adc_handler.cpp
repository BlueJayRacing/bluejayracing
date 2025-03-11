#include "adc_handler.hpp"

namespace baja {
namespace adc {

// Initialize static member
ADC7175Handler* ADC7175Handler::instance_ = nullptr;

ADC7175Handler::ADC7175Handler(buffer::RingBuffer<data::ChannelSample, RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      csPin_(0),
      drdyPin_(0),
      spiInterface_(nullptr),
      activeChannel_(0),
      sampleCount_(0),
      samplingActive_(false) {
    // Initialize channel configs to nullptr
    channelConfigs_ = nullptr;
}

ADC7175Handler::~ADC7175Handler() {
    // Make sure sampling is stopped before destruction
    stopSampling();
    
    // Remove this instance from the static pointer if it's the current one
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

bool ADC7175Handler::begin(uint8_t csPin, uint8_t drdyPin, SPIClass& spiInterface, 
                          const ADCSettings& settings) {
    Serial.println("  ADC Begin: Setting up pins and SPI");
    csPin_ = csPin;
    drdyPin_ = drdyPin;
    spiInterface_ = &spiInterface;
    
    // Configure the CS pin as output
    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH); // Deselect ADC
    
    // Configure the DRDY pin as input
    pinMode(drdyPin_, INPUT_PULLUP);
    
    Serial.println("  ADC Begin: Creating initialization parameters");
    // Create the initialization parameters
    ad717x_init_param_t initParam;
    initParam.active_device = settings.deviceType;
    initParam.mode = settings.operatingMode;
    initParam.stat_on_read_en = settings.readStatusWithData;
    initParam.ref_en = (settings.referenceSource == INTERNAL_REF);
    
    // Set up the channel map initially with all channels disabled
    Serial.println("  ADC Begin: Setting up channel map");
    initParam.chan_map.resize(ADC_CHANNEL_COUNT);
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        initParam.chan_map[i].channel_enable = false;
        initParam.chan_map[i].setup_sel = 0;
        
        // Default to AIN0 for positive input and AIN1 for negative
        initParam.chan_map[i].inputs.ainp.pos_input = static_cast<ad717x_analog_input_t>(i % 16);
        initParam.chan_map[i].inputs.ainp.neg_input = AIN1;
    }
    
    // Set up one default setup (we'll configure more when channels are added)
    Serial.println("  ADC Begin: Setting up default configuration");
    ad717x_setup_t setup;
    setup.setup.bi_polar = true;
    setup.setup.input_buff = true;
    setup.setup.ref_buff = true;
    setup.setup.ref_source = settings.referenceSource;
    setup.filter_config.odr = settings.odrSetting;
    setup.gain = 1.0;
    
    // Add the setup to the parameters
    initParam.setups.push_back(setup);
    
    // Ensure SPI is properly initialized before using it
    Serial.println("  ADC Begin: Initializing SPI");
    spiInterface_->begin();
    
    // Initialize the ADC with debugging
    Serial.println("  ADC Begin: Calling AD717X init function");
    int result = adcDriver_.init(initParam, spiInterface_, csPin_);
    Serial.print("  ADC init result: ");
    Serial.println(result);
    
    if (result < 0) {
        Serial.println("  ADC initialization failed with error code: " + String(result));
        return false;
    }
    
    // Set up the interrupt handler
    Serial.println("  ADC Begin: Setting up interrupt handler");
    setInterruptHandler(this);
    
    Serial.println("  ADC Begin: Initialization complete");
    return true;
}

bool ADC7175Handler::configureChannels(const ChannelConfig* configs, size_t numChannels) {
    Serial.print("Configuring ");
    Serial.print(numChannels);
    Serial.println(" ADC channels");
    
    // Store pointer to configurations
    channelConfigs_ = const_cast<ChannelConfig*>(configs);
    
    // Configure each channel
    for (size_t i = 0; i < numChannels; i++) {
        Serial.print("  Configuring channel ");
        Serial.print(i);
        Serial.print(" (index ");
        Serial.print(configs[i].channelIndex);
        Serial.println(")");
        
        if (!configureChannel(configs[i])) {
            Serial.print("  Failed to configure channel ");
            Serial.println(i);
            return false;
        }
    }
    
    Serial.println("All channels configured successfully");
    return true;
}

bool ADC7175Handler::configureChannel(const ChannelConfig& config) {
    // Set the channel status (enabled/disabled)
    Serial.print("  Setting channel ");
    Serial.print(config.channelIndex);
    Serial.print(" status: ");
    Serial.println(config.enabled ? "enabled" : "disabled");
    
    int result = adcDriver_.setChannelStatus(config.channelIndex, config.enabled);
    if (result < 0) {
        Serial.print("  setChannelStatus failed with code: ");
        Serial.println(result);
        return false;
    }
    
    // Connect the analog inputs
    Serial.print("  Connecting analog inputs for channel ");
    Serial.println(config.channelIndex);
    result = adcDriver_.connectAnalogInput(config.channelIndex, config.analogInputs);
    if (result < 0) {
        Serial.print("  connectAnalogInput failed with code: ");
        Serial.println(result);
        return false;
    }
    
    // Assign the setup
    Serial.print("  Assigning setup ");
    Serial.print(config.setupIndex);
    Serial.print(" to channel ");
    Serial.println(config.channelIndex);
    result = adcDriver_.assignSetup(config.channelIndex, config.setupIndex);
    if (result < 0) {
        Serial.print("  assignSetup failed with code: ");
        Serial.println(result);
        return false;
    }
    
    // Set the gain
    Serial.print("  Setting gain ");
    Serial.print(config.gain);
    Serial.print(" for setup ");
    Serial.println(config.setupIndex);
    result = adcDriver_.setGain(config.gain, config.setupIndex);
    if (result < 0) {
        Serial.print("  setGain failed with code: ");
        Serial.println(result);
        return false;
    }
    
    return true;
}

bool ADC7175Handler::startSampling() {
    // Check if we're already sampling
    Serial.println("Starting ADC sampling");
    if (samplingActive_) {
        Serial.println("Sampling already active");
        return true;
    }
    
    // Reset the sample count
    sampleCount_ = 0;
    
    // Set the ADC to continuous conversion mode
    Serial.println("  Setting ADC to CONTINUOUS mode");
    int result = adcDriver_.setADCMode(CONTINUOUS);
    if (result < 0) {
        Serial.print("  setADCMode failed with code: ");
        Serial.println(result);
        return false;
    }
    
    // Start sampling
    samplingActive_ = true;
    
    // Attach the interrupt to the DRDY pin
    Serial.print("  Attaching interrupt to pin ");
    Serial.println(drdyPin_);
    attachInterrupt(digitalPinToInterrupt(drdyPin_), isr, FALLING);
    
    Serial.println("Sampling started successfully");
    return true;
}

bool ADC7175Handler::stopSampling() {
    // Check if we're already stopped
    if (!samplingActive_) {
        return true;
    }
    
    // Detach the interrupt
    detachInterrupt(digitalPinToInterrupt(drdyPin_));
    
    // Set the ADC to standby mode
    int result = adcDriver_.setADCMode(STANDBY);
    if (result < 0) {
        return false;
    }
    
    // Mark as stopped
    samplingActive_ = false;
    
    return true;
}

bool ADC7175Handler::readSample(ad717x_data_t& sample) {
    // Read a sample from the ADC
    int result = adcDriver_.contConvReadData(&sample);
    
    // Update the active channel
    activeChannel_ = sample.status.active_channel;
    
    return (result >= 0);
}

uint8_t ADC7175Handler::getActiveChannel() const {
    return activeChannel_;
}

void ADC7175Handler::setInterruptPriority(uint8_t priority) {
    // Set the priority of the DRDY interrupt
    int irq = digitalPinToInterrupt(drdyPin_);
    NVIC_SET_PRIORITY(irq, priority);
    Serial.print("Set interrupt priority to ");
    Serial.print(priority);
    Serial.print(" for IRQ ");
    Serial.println(irq);
}

std::vector<ChannelConfig> ADC7175Handler::getChannelConfigs() const {
    // Create a temporary vector for the result
    std::vector<ChannelConfig> configs;
    
    // Check if channelConfigs_ is initialized
    if (channelConfigs_ == nullptr) {
        Serial.println("WARNING: channelConfigs_ is NULL in getChannelConfigs()");
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

void ADC7175Handler::handleInterrupt() {
    // Read the sample from the ADC
    ad717x_data_t sample;
    if (readSample(sample)) {
        // Create a new channel sample
        data::ChannelSample channelSample(
            micros(),                // Microsecond timestamp
            sample.status.active_channel,  // Channel index
            sample.value             // Raw ADC value
        );
        
        // Add to the ring buffer
        ringBuffer_.writeFromISR(channelSample);
        
        // Increment the sample count
        sampleCount_++;
    }
}

void ADC7175Handler::setInterruptHandler(ADC7175Handler* instance) {
    instance_ = instance;
}

// Static ISR that calls the instance method
void FASTRUN ADC7175Handler::isr() {
    if (instance_) {
        instance_->handleInterrupt();
    }
}

} // namespace adc
} // namespace baja