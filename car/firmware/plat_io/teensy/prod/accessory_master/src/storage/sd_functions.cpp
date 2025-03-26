#include "storage/sd_functions.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace storage {
namespace functions {

// Static variables to maintain state
static SDWriter* sdWriter_ = nullptr;
static bool running_ = false;
static uint64_t totalSamplesWritten_ = 0;

// Timing statistics
static uint32_t totalProcessingTime_ = 0;
static uint32_t minProcessingTime_ = UINT32_MAX;
static uint32_t maxProcessingTime_ = 0;
static uint32_t processingCount_ = 0;
static uint32_t lastStatResetTime_ = 0;

// Minimum samples needed for processing
static const size_t MIN_SAMPLES_FOR_PROCESSING = 15;

bool initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer,
    RingBuf<FsFile, config::SD_RING_BUF_CAPACITY>* sdRingBuf,
    uint8_t chipSelect) {
    
    util::Debug::info(F("SD: Initializing"));
    
    // Create the SD writer
    sdWriter_ = new SDWriter(ringBuffer, sdRingBuf);
    
    if (!sdWriter_) {
        util::Debug::error(F("SD: Failed to create SD writer"));
        return false;
    }
    
    // Initialize SD card
    if (!sdWriter_->begin(chipSelect)) {
        util::Debug::error(F("SD: SD card initialization failed"));
        return false;
    }
    
    // Reset timing statistics
    resetTimingStats();
    
    util::Debug::info(F("SD: Initialization successful"));
    return true;
}

bool start() {
    // Check if already running
    if (running_) {
        util::Debug::warning(F("SD: Already running"));
        return true;
    }
    
    // Check if SD writer is initialized
    if (!sdWriter_) {
        util::Debug::error(F("SD: SD writer not initialized"));
        return false;
    }
    
    // Create a new data file
    if (!sdWriter_->createNewFile()) {
        util::Debug::error(F("SD: Failed to create initial data file!"));
        return false;
    }
    
    util::Debug::info(F("SD: Created initial file: ") + 
                  String(sdWriter_->getCurrentFilename().c_str()));
    
    // Reset counters
    totalSamplesWritten_ = 0;
    
    // Reset timing statistics
    resetTimingStats();
    
    running_ = true;
    util::Debug::info(F("SD: Started"));
    return true;
}

bool stop() {
    if (!running_) {
        return true;
    }
    
    // Close the file
    if (sdWriter_) {
        sdWriter_->closeFile();
    }
    
    running_ = false;
    util::Debug::info(F("SD: Stopped"));
    
    return true;
}

bool isRunning() {
    return running_;
}

size_t process() {
    // Check if SD is running
    if (!sdWriter_ || !running_) {
        return 0;
    }
    
    // Start timing
    uint32_t startTime = micros();
    
    // Process SD operations - returns number of samples written
    size_t samplesWritten = sdWriter_->process();
    
    // Only update timing statistics if we actually did work (wrote samples)
    if (samplesWritten > 0) {
        uint32_t processingTime = micros() - startTime;
        
        // Update statistics
        totalProcessingTime_ += processingTime;
        processingCount_++;
        totalSamplesWritten_ += samplesWritten;
        
        if (processingTime < minProcessingTime_) {
            minProcessingTime_ = processingTime;
        }
        
        if (processingTime > maxProcessingTime_) {
            maxProcessingTime_ = processingTime;
        }
        
        // Log detailed info for large batches
        if (samplesWritten >= 50) {
            util::Debug::detail(F("SD: Processed ") + String(samplesWritten) + 
                            F(" samples in ") + String(processingTime) + F("µs"));
        }
    }
    
    return samplesWritten;
}

SDWriter* getWriter() {
    return sdWriter_;
}

void setChannelConfigs(const std::vector<adc::ChannelConfig>& channelConfigs) {
    if (!sdWriter_) {
        util::Debug::error(F("SD: SD writer not initialized"));
        return;
    }
    
    sdWriter_->setChannelNames(channelConfigs);
}

bool createNewFile(bool addHeader) {
    if (!sdWriter_) {
        util::Debug::error(F("SD: SD writer not initialized"));
        return false;
    }
    
    return sdWriter_->createNewFile(addHeader);
}

bool closeFile() {
    if (!sdWriter_) {
        util::Debug::error(F("SD: SD writer not initialized"));
        return false;
    }
    
    return sdWriter_->closeFile();
}

bool flush() {
    if (!sdWriter_) {
        util::Debug::error(F("SD: SD writer not initialized"));
        return false;
    }
    
    return sdWriter_->flush();
}

std::string getCurrentFilename() {
    if (!sdWriter_) {
        return "";
    }
    
    return sdWriter_->getCurrentFilename();
}

size_t getBytesWritten() {
    if (!sdWriter_) {
        return 0;
    }
    
    return sdWriter_->getBytesWritten();
}

uint64_t getSamplesWritten() {
    return totalSamplesWritten_;
}

void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint32_t& totalWrites) {
    avgTime = processingCount_ > 0 ? (float)totalProcessingTime_ / processingCount_ : 0.0f;
    minTime = minProcessingTime_ == UINT32_MAX ? 0 : minProcessingTime_;
    maxTime = maxProcessingTime_;
    totalWrites = processingCount_;
}

void resetTimingStats() {
    totalProcessingTime_ = 0;
    minProcessingTime_ = UINT32_MAX;
    maxProcessingTime_ = 0;
    processingCount_ = 0;
    lastStatResetTime_ = millis();
}

} // namespace functions
} // namespace storage
} // namespace baja