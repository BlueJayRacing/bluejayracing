#include "adc/adc_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace adc {

// Initialize static class members
ADC7175Handler* ADCThread::adcHandler_ = nullptr;
int ADCThread::threadId_ = -1;
volatile bool ADCThread::running_ = false;
uint64_t ADCThread::sampleCount_ = 0;

bool ADCThread::initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
    uint8_t csPin,
    SPIClass& spiInterface,
    const ADCSettings& settings) {
    
    util::Debug::info("ADC Thread: Initializing");
    
    // Create the ADC handler
    adcHandler_ = new ADC7175Handler(ringBuffer);
    
    if (!adcHandler_) {
        util::Debug::error("ADC Thread: Failed to create ADC handler");
        return false;
    }
    
    // Initialize the ADC hardware
    bool result = adcHandler_->begin(csPin, spiInterface, settings);
    
    if (!result) {
        util::Debug::error("ADC Thread: ADC initialization failed");
        
        // Try a reset and reinitialize
        adcHandler_->resetADC();
        delay(50);
        util::Debug::info("ADC Thread: Retrying ADC initialization...");
        result = adcHandler_->begin(csPin, spiInterface, settings);
        
        if (!result) {
            util::Debug::error("ADC Thread: ADC retry failed");
            return false;
        }
    }
    
    util::Debug::info("ADC Thread: Initialization successful");
    return true;
}

int ADCThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("ADC Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Check if ADC handler is initialized
    if (!adcHandler_) {
        util::Debug::error("ADC Thread: ADC handler not initialized");
        return -1;
    }
    
    // Start ADC sampling
    if (!adcHandler_->startSampling()) {
        util::Debug::error("ADC Thread: Failed to start ADC sampling");
        return -2;
    }
    
    // Reset sample count
    sampleCount_ = 0;
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 4096);
    
    if (threadId_ <= 0) {
        util::Debug::error("ADC Thread: Failed to create thread");
        adcHandler_->stopSampling();
        running_ = false;
        return -3;
    }
    
    util::Debug::info("ADC Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool ADCThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    
    // Stop ADC sampling
    if (adcHandler_) {
        adcHandler_->stopSampling();
    }
    
    threadId_ = -1;
    util::Debug::info("ADC Thread: Stopped");
    
    return true;
}

bool ADCThread::isRunning() {
    return running_ && threadId_ > 0;
}

ADC7175Handler* ADCThread::getHandler() {
    return adcHandler_;
}

bool ADCThread::configureChannels(const ChannelConfig* configs, size_t numChannels) {
    if (!adcHandler_) {
        util::Debug::error("ADC Thread: ADC handler not initialized");
        return false;
    }
    
    return adcHandler_->configureChannels(configs, numChannels);
}

bool ADCThread::processSample() {
    if (!adcHandler_) {
        return false;
    }
    
    // Poll for new sample (non-blocking)
    if (adcHandler_->pollForSample(0)) {
        sampleCount_++;
        return true;
    }
    
    return false;
}

void ADCThread::threadFunction(void* arg) {
    util::Debug::info("ADC Thread: Thread started");
    
    // Set thread priority
    threads.setTimeSlice(threads.id(), 5);  // Higher priority
    
    // Main thread loop
    while (running_) {
        // Process ADC data using polling approach
        if (adcHandler_) {
            // Poll for new sample (non-blocking)
            adcHandler_->pollForSample(0);
        }
        
        // Small yield for other tasks
        threads.yield();
    }
    
    util::Debug::info("ADC Thread: Thread ended");
}

} // namespace adc
} // namespace baja