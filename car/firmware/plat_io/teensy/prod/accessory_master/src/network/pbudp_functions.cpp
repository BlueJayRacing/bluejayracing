#include "network/pbudp_functions.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace network {
namespace functions {

// Static variables to maintain state
static PBUDPHandler* handler_ = nullptr;
static bool running_ = false;

// Timing statistics
static uint32_t totalProcessingTime_ = 0;
static uint32_t minProcessingTime_ = UINT32_MAX;
static uint32_t maxProcessingTime_ = 0;
static uint32_t processingCount_ = 0;
static uint32_t lastStatResetTime_ = 0;

// Minimum samples needed for processing
static const size_t MIN_SAMPLES_FOR_PROCESSING = config::FIXED_SAMPLE_COUNT;

bool initialize(
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer,
    const char* serverAddress,
    uint16_t port) {
    
    util::Debug::info(F("PBUDP: Initializing"));
    
    // Create the handler
    handler_ = new PBUDPHandler(sourceBuffer);
    
    if (!handler_) {
        util::Debug::error(F("PBUDP: Failed to create handler"));
        return false;
    }
    
    // Initialize handler with server address and port
    if (!handler_->initialize(serverAddress, port)) {
        util::Debug::error(F("PBUDP: Handler initialization failed"));
        return false;
    }
    
    // Reset timing statistics
    resetTimingStats();
    
    util::Debug::info(F("PBUDP: Initialization successful"));
    return true;
}

bool start() {
    // Check if already running
    if (running_) {
        util::Debug::warning(F("PBUDP: Already running"));
        return true;
    }
    
    // Check if handler is initialized
    if (!handler_) {
        util::Debug::error(F("PBUDP: Handler not initialized"));
        return false;
    }
    
    // Reset timing statistics
    resetTimingStats();
    
    running_ = true;
    util::Debug::info(F("PBUDP: Started"));
    return true;
}

bool stop() {
    if (!running_) {
        return true;
    }
    
    running_ = false;
    util::Debug::info(F("PBUDP: Stopped"));
    
    return true;
}

bool isRunning() {
    return running_;
}

size_t process() {
    // Check if PBUDP is running
    if (!handler_ || !running_) {
        return 0;
    }
    
    // Start timing
    uint32_t startTime = micros();
    
    // Process and send a batch of samples
    // PBUDPHandler::processAndSendBatch already has threshold checks
    size_t samplesProcessed = handler_->processAndSendBatch();
    
    // Only update timing statistics if we actually did work (sent samples)
    if (samplesProcessed > 0) {
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
        
        // Log detailed info for large batches or slow processing
        if (samplesProcessed > 10 || processingTime > 1000) {
            util::Debug::detail(F("PBUDP: Processed ") + String(samplesProcessed) + 
                            F(" samples in ") + String(processingTime) + F("µs"));
        }
    }
    
    return samplesProcessed;
}

PBUDPHandler* getHandler() {
    return handler_;
}

void getStats(uint32_t& messagesSent, uint32_t& sampleCount, 
            uint32_t& bytesTransferred, uint32_t& sendErrors) {
    if (handler_) {
        messagesSent = handler_->getMessagesSent();
        sampleCount = handler_->getSampleCount();
        bytesTransferred = handler_->getBytesTransferred();
        sendErrors = handler_->getSendErrors();
    } else {
        messagesSent = 0;
        sampleCount = 0;
        bytesTransferred = 0;
        sendErrors = 0;
    }
}

void getTimingStats(float& avgTime, uint32_t& minTime, uint32_t& maxTime, uint32_t& messageCount) {
    avgTime = processingCount_ > 0 ? (float)totalProcessingTime_ / processingCount_ : 0.0f;
    minTime = minProcessingTime_ == UINT32_MAX ? 0 : minProcessingTime_;
    maxTime = maxProcessingTime_;
    messageCount = processingCount_;
}

void resetTimingStats() {
    totalProcessingTime_ = 0;
    minProcessingTime_ = UINT32_MAX;
    maxProcessingTime_ = 0;
    processingCount_ = 0;
    lastStatResetTime_ = millis();
}

} // namespace functions
} // namespace network
} // namespace baja