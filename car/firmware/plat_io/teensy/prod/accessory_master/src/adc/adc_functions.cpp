#include "adc/adc_functions.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace adc {
namespace functions {

// Static variables to maintain state
static ADC7175Handler* adcHandler_ = nullptr;
static bool running_ = false;
static uint64_t sampleCount_ = 0;
static buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* fastBuffer_ = nullptr;
static uint16_t channelSampleCounters_[util::TOTAL_CHANNEL_COUNT] = {0};

// Timing statistics
static uint32_t totalProcessingTime_ = 0;
static uint32_t minProcessingTime_ = UINT32_MAX;
static uint32_t maxProcessingTime_ = 0;
static uint32_t processingCount_ = 0;
static uint32_t lastStatResetTime_ = 0;

bool initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& mainBuffer,
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer,
    uint8_t csPin,
    SPIClass& spiInterface,
    const ADCSettings& settings) {
    
    util::Debug::info(F("ADC: Initializing"));
    
    // Store the fast buffer reference
    fastBuffer_ = &fastBuffer;
    
    // Reset all channel sample counters
    for (int i = 0; i < util::TOTAL_CHANNEL_COUNT; i++) {
        channelSampleCounters_[i] = 0;
    }
    
    // Create the ADC handler
    adcHandler_ = new ADC7175Handler(mainBuffer);
    
    if (!adcHandler_) {
        util::Debug::error(F("ADC: Failed to create ADC handler"));
        return false;
    }
    
    // Initialize the ADC hardware
    bool result = adcHandler_->begin(csPin, spiInterface, settings);
    
    if (!result) {
        util::Debug::error(F("ADC: ADC initialization failed"));
        
        // Try a reset and reinitialize
        adcHandler_->resetADC();
        delay(50);
        util::Debug::info(F("ADC: Retrying ADC initialization..."));
        result = adcHandler_->begin(csPin, spiInterface, settings);
        
        if (!result) {
            util::Debug::error(F("ADC: ADC retry failed"));
            return false;
        }
    }
    
    // Reset timing statistics
    resetTimingStats();
    
    util::Debug::info(F("ADC: Initialization successful"));
    return true;
}

bool start() {
    // Check if already running
    if (running_) {
        util::Debug::warning(F("ADC: Already running"));
        return true;
    }
    
    // Check if ADC handler is initialized
    if (!adcHandler_) {
        util::Debug::error(F("ADC: ADC handler not initialized"));
        return false;
    }
    
    // Start ADC sampling
    if (!adcHandler_->startSampling()) {
        util::Debug::error(F("ADC: Failed to start ADC sampling"));
        return false;
    }
    
    // Reset sample count and channel counters
    sampleCount_ = 0;
    for (int i = 0; i < util::TOTAL_CHANNEL_COUNT; i++) {
        channelSampleCounters_[i] = 0;
    }
    
    // Reset timing statistics
    resetTimingStats();
    
    running_ = true;
    util::Debug::info(F("ADC: Started"));
    return true;
}

bool stop() {
    if (!running_) {
        return true;
    }
    
    // Stop ADC sampling
    if (adcHandler_) {
        adcHandler_->stopSampling();
    }
    
    running_ = false;
    util::Debug::info(F("ADC: Stopped"));
    
    return true;
}

bool isRunning() {
    return running_;
}

bool processSample() {
    // Check if ADC is running
    if (!adcHandler_ || !running_) {
        return false;
    }
    
    // Start timing for this operation
    uint32_t startTime = micros();
    
    // Poll for new sample (non-blocking)
    ad717x_data_t adcSample;
    bool sampleProcessed = false;
    int ret = adcHandler_->pollForSample(0);
    if ( ret == AH_BUFFERBAD || ret == AH_OK ) {
        // Get the actual sample data
        if (adcHandler_->getLatestConversion(adcSample)) {
            // IMPORTANT: Convert ADC channel index to internal channel ID
            uint8_t internalChannelId = static_cast<uint8_t>(
                util::mapADCToInternalID(adcSample.status.active_channel));
            
            // Create a channel sample with internal ID and recorded time
            data::ChannelSample channelSample(
                micros(),                // Microsecond timestamp
                internalChannelId,       // Internal channel ID (converted from ADC channel)
                adcSample.value,         // Raw ADC value
                millis()                 // Add recorded time in milliseconds
            );
            
            // Write to fast buffer with downsampling
            if (fastBuffer_) {
                if (internalChannelId < util::TOTAL_CHANNEL_COUNT) {
                    // Increment channel counter
                    channelSampleCounters_[internalChannelId]++;
                    
                    // Every N samples, write to fast buffer
                    if (channelSampleCounters_[internalChannelId] >= config::FAST_BUFFER_DOWNSAMPLE_RATIO) {
                        // Reset counter
                        channelSampleCounters_[internalChannelId] = 0;
                        
                        // Write to fast buffer (this will always succeed due to overwrite policy)
                        fastBuffer_->write(channelSample);
                    }
                }
            }
            
            // Update sample count and completed flag
            if ( ret == AH_OK ) {
                sampleCount_++;
                sampleProcessed = true;
            }
            
        }
    }
    
    // Calculate processing time if we actually did work
    if (sampleProcessed) {
        uint32_t processingTime = micros() - startTime;
        
        // Update statistics
        totalProcessingTime_ += processingTime;
        processingCount_++;
        
        if (processingTime < minProcessingTime_) {
            minProcessingTime_ = processingTime;
        }
        
        if (processingTime > maxProcessingTime_) {
            maxProcessingTime_ = processingTime;
        }
        
        // Log extremely long processing times
        if (processingTime > 100) {
            // Avoid log spam by only logging long times occasionally
            static uint32_t lastLogTime = 0;
            uint32_t currentTime = millis();
            
            if (currentTime - lastLogTime > 5000) {
                util::Debug::warning(F("ADC: Long processing time: ") + String(processingTime) + F("µs"));
                lastLogTime = currentTime;
            }
        }
    }
    
    return sampleProcessed;
}

bool configureChannels(const ChannelConfig* configs, size_t numChannels) {
    if (!adcHandler_) {
        util::Debug::error(F("ADC: ADC handler not initialized"));
        return false;
    }
    
    return adcHandler_->configureChannels(configs, numChannels);
}

ADC7175Handler* getHandler() {
    return adcHandler_;
}

buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* getFastBuffer() {
    return fastBuffer_;
}

void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint64_t& sampleCount) {
    avgTime = processingCount_ > 0 ? (float)totalProcessingTime_ / processingCount_ : 0.0f;
    minTime = minProcessingTime_ == UINT32_MAX ? 0 : minProcessingTime_;
    maxTime = maxProcessingTime_;
    sampleCount = sampleCount_;
}

void resetTimingStats() {
    totalProcessingTime_ = 0;
    minProcessingTime_ = UINT32_MAX;
    maxProcessingTime_ = 0;
    processingCount_ = 0;
    lastStatResetTime_ = millis();
}

uint64_t getSampleCount() {
    return sampleCount_;
}

uint8_t getActiveChannel() {
    return adcHandler_ ? adcHandler_->getActiveChannel() : 0;
}

uint8_t getActiveInternalChannelId() {
    if (!adcHandler_) return 0;
    
    uint8_t adcChannel = adcHandler_->getActiveChannel();
    return static_cast<uint8_t>(util::mapADCToInternalID(adcChannel));
}

} // namespace functions
} // namespace adc
} // namespace baja