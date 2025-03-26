// #include "adc/adc_thread.hpp"
// #include "util/debug_util.hpp"

// namespace baja {
// namespace adc {

// // Initialize static class members
// ADC7175Handler* ADCThread::adcHandler_ = nullptr;
// int ADCThread::threadId_ = -1;
// volatile bool ADCThread::running_ = false;
// uint64_t ADCThread::sampleCount_ = 0;
// buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* ADCThread::fastBuffer_ = nullptr;
// uint16_t ADCThread::channelSampleCounters_[ADC_CHANNEL_COUNT] = {0};

// bool ADCThread::initialize(
//     buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
//     buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer,
//     uint8_t csPin,
//     SPIClass& spiInterface,
//     const ADCSettings& settings) {
    
//     util::Debug::info("ADC Thread: Initializing");
    
//     // Store the fast buffer reference
//     fastBuffer_ = &fastBuffer;
    
//     // Reset all channel sample counters
//     for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
//         channelSampleCounters_[i] = 0;
//     }
    
//     // Create the ADC handler
//     adcHandler_ = new ADC7175Handler(ringBuffer);
    
//     if (!adcHandler_) {
//         util::Debug::error("ADC Thread: Failed to create ADC handler");
//         return false;
//     }
    
//     // Initialize the ADC hardware
//     bool result = adcHandler_->begin(csPin, spiInterface, settings);
    
//     if (!result) {
//         util::Debug::error("ADC Thread: ADC initialization failed");
        
//         // Try a reset and reinitialize
//         adcHandler_->resetADC();
//         delay(50);
//         util::Debug::info("ADC Thread: Retrying ADC initialization...");
//         result = adcHandler_->begin(csPin, spiInterface, settings);
        
//         if (!result) {
//             util::Debug::error("ADC Thread: ADC retry failed");
//             return false;
//         }
//     }
    
//     util::Debug::info("ADC Thread: Initialization successful");
//     return true;
// }

// int ADCThread::start() {
//     // Check if already running
//     if (running_ || threadId_ > 0) {
//         util::Debug::warning("ADC Thread: Already running with ID " + String(threadId_));
//         return threadId_;
//     }
    
//     // Check if ADC handler is initialized
//     if (!adcHandler_) {
//         util::Debug::error("ADC Thread: ADC handler not initialized");
//         return -1;
//     }
    
//     // Start ADC sampling
//     if (!adcHandler_->startSampling()) {
//         util::Debug::error("ADC Thread: Failed to start ADC sampling");
//         return -2;
//     }
    
//     // Reset sample count and channel counters
//     sampleCount_ = 0;
//     for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
//         channelSampleCounters_[i] = 0;
//     }
    
//     // Create the thread
//     running_ = true;
//     threadId_ = threads.addThread(threadFunction, 0, 4096);
    
//     if (threadId_ <= 0) {
//         util::Debug::error("ADC Thread: Failed to create thread");
//         adcHandler_->stopSampling();
//         running_ = false;
//         return -3;
//     }
    
//     // Set thread priority
//     threads.setTimeSlice(threadId_, config::ADC_THREAD_PRIORITY);
    
//     util::Debug::info("ADC Thread: Started with ID " + String(threadId_));
//     return threadId_;
// }

// bool ADCThread::stop() {
//     if (!running_ || threadId_ <= 0) {
//         return false;
//     }
    
//     // Signal thread to stop
//     running_ = false;
    
//     // Wait for thread to finish
//     threads.wait(threadId_, 1000);  // Wait up to 1 second
    
//     // Stop ADC sampling
//     if (adcHandler_) {
//         adcHandler_->stopSampling();
//     }
    
//     threadId_ = -1;
//     util::Debug::info("ADC Thread: Stopped");
    
//     return true;
// }

// bool ADCThread::isRunning() {
//     return running_ && threadId_ > 0;
// }

// ADC7175Handler* ADCThread::getHandler() {
//     return adcHandler_;
// }

// buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* ADCThread::getFastBuffer() {
//     return fastBuffer_;
// }

// bool ADCThread::configureChannels(const ChannelConfig* configs, size_t numChannels) {
//     if (!adcHandler_) {
//         util::Debug::error("ADC Thread: ADC handler not initialized");
//         return false;
//     }
    
//     return adcHandler_->configureChannels(configs, numChannels);
// }

// bool ADCThread::processSample() {
//     if (!adcHandler_) {
//         return false;
//     }
    
//     // Poll for new sample (non-blocking)
//     ad717x_data_t adcSample;
//     if (adcHandler_->readSample(adcSample)) {
//         // Create a channel sample
//         data::ChannelSample channelSample(
//             micros(),                   // Microsecond timestamp
//             adcSample.status.active_channel, // Channel index
//             adcSample.value                // Raw ADC value
//         );
        
//         // Write to fast buffer with downsampling
//         if (fastBuffer_) {
//             uint8_t channelIndex = channelSample.channelIndex;
//             if (channelIndex < ADC_CHANNEL_COUNT) {
//                 // Increment channel counter
//                 channelSampleCounters_[channelIndex]++;
                
//                 // Every N samples, write to fast buffer
//                 if (channelSampleCounters_[channelIndex] >= config::FAST_BUFFER_DOWNSAMPLE_RATIO) {
//                     // Reset counter
//                     channelSampleCounters_[channelIndex] = 0;
                    
//                     // Write to fast buffer (this will always succeed due to overwrite policy)
//                     fastBuffer_->write(channelSample);
//                 }
//             }
//         }
        
//         // Update sample count
//         sampleCount_++;
//         return true;
//     }
    
//     return false;
// }

// void ADCThread::threadFunction(void* arg) {
//     util::Debug::info("ADC Thread: Thread started");
    
//     // Main thread loop
//     while (running_) {
//         // Process ADC data using polling approach
//         if (adcHandler_) {
//             // Poll for new sample (non-blocking)
//             ad717x_data_t adcSample;
//             if (adcHandler_->pollForSample(0)) {
//                 // Get the data - note that the ADC handler automatically adds to main buffer
                
//                 // For fast buffer, we need to manually create the sample and add it 
//                 // with downsampling if needed
//                 if (fastBuffer_) {
//                     // Create a channel sample
//                     data::ChannelSample channelSample(
//                         micros(),                   // Microsecond timestamp
//                         adcSample.status.active_channel, // Channel index
//                         adcSample.value                // Raw ADC value
//                     );
                    
//                     // Apply channel-specific downsampling
//                     uint8_t channelIndex = channelSample.channelIndex;
//                     if (channelIndex < ADC_CHANNEL_COUNT) {
//                         // Increment channel counter
//                         channelSampleCounters_[channelIndex]++;
                        
//                         // Every N samples, write to fast buffer
//                         if (channelSampleCounters_[channelIndex] >= config::FAST_BUFFER_DOWNSAMPLE_RATIO) {
//                             // Reset counter
//                             channelSampleCounters_[channelIndex] = 0;
                            
//                             // Write to fast buffer (this will always succeed due to overwrite policy)
//                             fastBuffer_->write(channelSample);
//                         }
//                     }
//                 }
                
//                 // Update sample count
//                 sampleCount_++;
//             }
//         }
        
//         // Small yield for other tasks
//         threads.yield();
//     }
    
//     util::Debug::info("ADC Thread: Thread ended");
// }

// } // namespace adc
// } // namespace baja
#include "adc/adc_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace adc {

// Initialize static class members
ADC7175Handler* ADCThread::adcHandler_ = nullptr;
int ADCThread::threadId_ = -1;
volatile bool ADCThread::running_ = false;
uint64_t ADCThread::sampleCount_ = 0;
buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* ADCThread::fastBuffer_ = nullptr;
uint16_t ADCThread::channelSampleCounters_[ADC_CHANNEL_COUNT] = {0};

bool ADCThread::initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& fastBuffer,
    uint8_t csPin,
    SPIClass& spiInterface,
    const ADCSettings& settings) {
    
    util::Debug::info("ADC Thread: Initializing");
    
    // Store the fast buffer reference
    fastBuffer_ = &fastBuffer;
    
    // Reset all channel sample counters
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        channelSampleCounters_[i] = 0;
    }
    
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
    
    // Reset sample count and channel counters
    sampleCount_ = 0;
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        channelSampleCounters_[i] = 0;
    }
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 4096);
    
    if (threadId_ <= 0) {
        util::Debug::error("ADC Thread: Failed to create thread");
        adcHandler_->stopSampling();
        running_ = false;
        return -3;
    }
    
    // Set thread priority
    threads.setTimeSlice(threadId_, config::ADC_THREAD_PRIORITY);
    
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

buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>* ADCThread::getFastBuffer() {
    return fastBuffer_;
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
    
    // Local static variables for timing statistics
    static uint32_t lastStatsTime = 0;
    static uint32_t minProcessingTime = UINT32_MAX;
    static uint32_t maxProcessingTime = 0;
    static uint64_t totalProcessingTime = 0;
    static uint32_t samplesSinceLastStats = 0;
    static const uint32_t statsIntervalMs = 10000; // 10 seconds
    
    // Start timing
    uint32_t startTime = micros();
    
    // Poll for new sample (non-blocking)
    ad717x_data_t adcSample;
    if (adcHandler_->readSample(adcSample)) {
        // Create a channel sample
        data::ChannelSample channelSample(
            micros(),                   // Microsecond timestamp
            adcSample.status.active_channel, // Channel index
            adcSample.value                // Raw ADC value
        );
        
        // Write to fast buffer with downsampling
        if (fastBuffer_) {
            uint8_t channelIndex = channelSample.channelIndex;
            if (channelIndex < ADC_CHANNEL_COUNT) {
                // Increment channel counter
                channelSampleCounters_[channelIndex]++;
                
                // Every N samples, write to fast buffer
                if (channelSampleCounters_[channelIndex] >= config::FAST_BUFFER_DOWNSAMPLE_RATIO) {
                    // Reset counter
                    channelSampleCounters_[channelIndex] = 0;
                    
                    // Write to fast buffer (this will always succeed due to overwrite policy)
                    fastBuffer_->write(channelSample);
                }
            }
        }
        
        // Update sample count
        sampleCount_++;
        
        // End timing and update statistics
        uint32_t processingTime = micros() - startTime;
        
        // Update timing stats
        if (processingTime < minProcessingTime) {
            minProcessingTime = processingTime;
        }
        if (processingTime > maxProcessingTime) {
            maxProcessingTime = processingTime;
        }
        totalProcessingTime += processingTime;
        samplesSinceLastStats++;
        
        // Log stats periodically
        uint32_t currentTime = millis();
        if (currentTime - lastStatsTime >= statsIntervalMs) {
            // Calculate average processing time
            float avgProcessingTime = (float)totalProcessingTime / samplesSinceLastStats;
            
            // Calculate samples per second
            float samplesPerSecond = (float)samplesSinceLastStats * 1000.0f / (currentTime - lastStatsTime);
            
            // Log statistics
            util::Debug::info("processSample() Timing: " + 
                            String(samplesSinceLastStats) + " samples in " + 
                            String((currentTime - lastStatsTime) / 1000.0f, 1) + " sec (" + 
                            String(samplesPerSecond, 1) + " samples/sec)");
            
            util::Debug::info("processSample() Times: avg=" + 
                            String(avgProcessingTime, 1) + "µs, min=" + 
                            String(minProcessingTime) + "µs, max=" + 
                            String(maxProcessingTime) + "µs");
            
            // Log fast buffer status if available
            if (fastBuffer_) {
                util::Debug::info("Fast Buffer: " + 
                                String(fastBuffer_->available()) + "/" + 
                                String(fastBuffer_->capacity()) + " samples (" + 
                                String((float)fastBuffer_->available() / fastBuffer_->capacity() * 100.0f, 1) + "%), " +
                                "Overwrites: " + String(fastBuffer_->getOverwriteCount()));
            }
            
            // Reset statistics
            lastStatsTime = currentTime;
            minProcessingTime = UINT32_MAX;
            maxProcessingTime = 0;
            totalProcessingTime = 0;
            samplesSinceLastStats = 0;
        }
        
        return true;
    }
    
    return false;
}

void ADCThread::threadFunction(void* arg) {
    util::Debug::info("ADC Thread: Thread started");
    
    // Local static variables for timing statistics
    static uint32_t lastStatsTime = 0;
    static uint32_t minProcessingTime = UINT32_MAX;
    static uint32_t maxProcessingTime = 0;
    static uint64_t totalProcessingTime = 0;
    static uint32_t samplesSinceLastStats = 0;
    static const uint32_t statsIntervalMs = 2000; // 10 seconds
    
    // Initialize timing statistics for this run
    lastStatsTime = millis();
    
    // Main thread loop
    while (running_) {
        // Start timing
        uint32_t startTime = micros();
        
        // Process ADC data using polling approach
        bool sampleProcessed = false;
        if (adcHandler_) {
            // Poll for new sample (non-blocking)
            ad717x_data_t adcSample;
            if (adcHandler_->pollForSample(0)) {
                sampleProcessed = true;
                
                // For fast buffer, we need to manually create the sample and add it 
                // with downsampling if needed
                if (fastBuffer_) {
                    // Create a channel sample
                    data::ChannelSample channelSample(
                        micros(),                   // Microsecond timestamp
                        adcSample.status.active_channel, // Channel index
                        adcSample.value                // Raw ADC value
                    );
                    
                    // Apply channel-specific downsampling
                    uint8_t channelIndex = channelSample.channelIndex;
                    if (channelIndex < ADC_CHANNEL_COUNT) {
                        // Increment channel counter
                        channelSampleCounters_[channelIndex]++;
                        
                        // Every N samples, write to fast buffer
                        if (channelSampleCounters_[channelIndex] >= config::FAST_BUFFER_DOWNSAMPLE_RATIO) {
                            // Reset counter
                            channelSampleCounters_[channelIndex] = 0;
                            
                            // Write to fast buffer (this will always succeed due to overwrite policy)
                            fastBuffer_->write(channelSample);
                        }
                    }
                }
                
                // Update sample count
                sampleCount_++;
                
                // End timing and update statistics
                uint32_t processingTime = micros() - startTime;
                
                // Update timing stats
                if (processingTime < minProcessingTime) {
                    minProcessingTime = processingTime;
                }
                if (processingTime > maxProcessingTime) {
                    maxProcessingTime = processingTime;
                }
                totalProcessingTime += processingTime;
                samplesSinceLastStats++;
            }
        }
        
        // Check if it's time to log statistics
        uint32_t currentTime = millis();
        if (currentTime - lastStatsTime >= statsIntervalMs) {
            // Only log if we have samples
            if (samplesSinceLastStats > 0) {
                // Calculate average processing time
                float avgProcessingTime = (float)totalProcessingTime / samplesSinceLastStats;
                
                // Calculate samples per second
                float samplesPerSecond = (float)samplesSinceLastStats * 1000.0f / (currentTime - lastStatsTime);
                
                // Log statistics
                util::Debug::info("ADC Thread Timing: " + 
                                String(samplesSinceLastStats) + " samples in " + 
                                String((currentTime - lastStatsTime) / 1000.0f, 1) + " sec (" + 
                                String(samplesPerSecond, 1) + " samples/sec)");
                
                util::Debug::info("ADC Thread Times: avg=" + 
                                String(avgProcessingTime, 1) + "µs, min=" + 
                                String(minProcessingTime) + "µs, max=" + 
                                String(maxProcessingTime) + "µs");
                
                // Log fast buffer status if available
                if (fastBuffer_) {
                    util::Debug::info("Fast Buffer: " + 
                                    String(fastBuffer_->available()) + "/" + 
                                    String(fastBuffer_->capacity()) + " samples (" + 
                                    String((float)fastBuffer_->available() / fastBuffer_->capacity() * 100.0f, 1) + "%), " +
                                    "Overwrites: " + String(fastBuffer_->getOverwriteCount()));
                }
            } else {
                util::Debug::warning("ADC Thread: No samples processed in the last " + 
                                  String(statsIntervalMs / 1000) + " seconds");
            }
            
            // Reset statistics
            lastStatsTime = currentTime;
            minProcessingTime = UINT32_MAX;
            maxProcessingTime = 0;
            totalProcessingTime = 0;
            samplesSinceLastStats = 0;
        }
        
        // Small yield for other tasks
        static int counter = 0;
        if (counter++ > 0) {
            threads.yield();
            counter = 0;
        }   
    }
    
    // Log final statistics if we have samples
    if (samplesSinceLastStats > 0) {
        uint32_t currentTime = millis();
        float avgProcessingTime = (float)totalProcessingTime / samplesSinceLastStats;
        float samplesPerSecond = (float)samplesSinceLastStats * 1000.0f / (currentTime - lastStatsTime);
        
        util::Debug::info("ADC Thread Final Stats: " + 
                        String(samplesSinceLastStats) + " samples (" + 
                        String(samplesPerSecond, 1) + " samples/sec), avg=" + 
                        String(avgProcessingTime, 1) + "µs, min=" + 
                        String(minProcessingTime) + "µs, max=" + 
                        String(maxProcessingTime) + "µs");
    }
    
    util::Debug::info("ADC Thread: Thread ended");
}

} // namespace adc
} // namespace baja