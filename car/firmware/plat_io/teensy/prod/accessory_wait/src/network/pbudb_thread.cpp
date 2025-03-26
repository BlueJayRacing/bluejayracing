#include "network/pbudp_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace network {

// Initialize static class members
PBUDPHandler* PBUDPThread::handler_ = nullptr;
int PBUDPThread::threadId_ = -1;
volatile bool PBUDPThread::running_ = false;

// Benchmarking metrics
uint32_t PBUDPThread::processingTimes_[100] = {0};
uint8_t PBUDPThread::timeIndex_ = 0;
uint32_t PBUDPThread::maxProcessingTime_ = 0;
uint32_t PBUDPThread::minProcessingTime_ = UINT32_MAX;
uint32_t PBUDPThread::totalProcessingTime_ = 0;
uint32_t PBUDPThread::totalBatches_ = 0;
uint32_t PBUDPThread::benchmarkStartTime_ = 0;

bool PBUDPThread::initialize(
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer,
    const char* serverAddress,
    uint16_t port) {
    
    util::Debug::info("PBUDP Thread: Initializing");
    
    // Create the handler
    handler_ = new PBUDPHandler(sourceBuffer);
    
    if (!handler_) {
        util::Debug::error("PBUDP Thread: Failed to create handler");
        return false;
    }
    
    // Initialize handler with server address and port
    if (!handler_->initialize(serverAddress, port)) {
        util::Debug::error("PBUDP Thread: Handler initialization failed");
        return false;
    }
    
    // Reset benchmark metrics
    timeIndex_ = 0;
    maxProcessingTime_ = 0;
    minProcessingTime_ = UINT32_MAX;
    totalProcessingTime_ = 0;
    totalBatches_ = 0;
    benchmarkStartTime_ = millis();
    
    util::Debug::info("PBUDP Thread: Initialization successful");
    return true;
}

int PBUDPThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("PBUDP Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Check if handler is initialized
    if (!handler_) {
        util::Debug::error("PBUDP Thread: Handler not initialized");
        return -1;
    }
    
    // Create the thread with high priority
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 108192); // more stack needed for serialization
    
    if (threadId_ <= 0) {
        util::Debug::error("PBUDP Thread: Failed to create thread");
        running_ = false;
        return -2;
    }
    
    // Set thread priority (higher number = higher priority in TeensyThreads)
    threads.setTimeSlice(threadId_, 7);
    
    util::Debug::info("PBUDP Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool PBUDPThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    threadId_ = -1;
    
    // Print final benchmarks
    printBenchmarks();
    
    util::Debug::info("PBUDP Thread: Stopped");
    return true;
}

bool PBUDPThread::isRunning() {
    return running_ && threadId_ > 0;
}

PBUDPHandler* PBUDPThread::getHandler() {
    return handler_;
}

void PBUDPThread::getStats(uint32_t& messagesSent, uint32_t& sampleCount, 
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

void PBUDPThread::recordProcessingTime(uint32_t processingTime) {
    // Store in circular buffer
    processingTimes_[timeIndex_] = processingTime;
    timeIndex_ = (timeIndex_ + 1) % 100;
    
    // Update stats
    totalProcessingTime_ += processingTime;
    totalBatches_++;
    
    if (processingTime > maxProcessingTime_) {
        maxProcessingTime_ = processingTime;
    }
    
    if (processingTime < minProcessingTime_) {
        minProcessingTime_ = processingTime;
    }
}

void PBUDPThread::printBenchmarks() {
    // Calculate elapsed time
    uint32_t elapsed = millis() - benchmarkStartTime_;
    if (elapsed == 0) elapsed = 1; // Avoid division by zero
    
    // Get current stats
    uint32_t messagesSent = 0, sampleCount = 0, bytesTransferred = 0, sendErrors = 0;
    getStats(messagesSent, sampleCount, bytesTransferred, sendErrors);
    
    // Calculate rates
    float msgsPerSec = messagesSent * 1000.0f / elapsed;
    float samplesPerSec = sampleCount * 1000.0f / elapsed;
    float kbytesPerSec = bytesTransferred * 1000.0f / elapsed / 1024.0f;
    
    // Calculate average processing time
    float avgProcessingTime = totalBatches_ > 0 ? 
                            totalProcessingTime_ / (float)totalBatches_ : 0;
    
    // Print benchmark results
    util::Debug::info("--------- PBUDP THREAD BENCHMARKS ---------");
    util::Debug::info("Runtime: " + String(elapsed / 1000.0f, 1) + " seconds");
    util::Debug::info("Messages sent: " + String(messagesSent) + 
                    " (" + String(msgsPerSec, 2) + " msgs/sec)");
    util::Debug::info("Samples processed: " + String(sampleCount) + 
                    " (" + String(samplesPerSec, 2) + " samples/sec)");
    util::Debug::info("Data transferred: " + String(bytesTransferred / 1024.0f, 2) + 
                    " KB (" + String(kbytesPerSec, 2) + " KB/sec)");
    
    if (totalBatches_ > 0) {
        util::Debug::info("Processing time: avg=" + String(avgProcessingTime, 2) + 
                        "µs, min=" + String(minProcessingTime_) + 
                        "µs, max=" + String(maxProcessingTime_) + "µs");
    }
    
    if (sendErrors > 0) {
        util::Debug::warning("Send errors: " + String(sendErrors));
    }
    
    util::Debug::info("------------------------------------------");
}

void PBUDPThread::threadFunction(void* arg) {
    util::Debug::info("PBUDP Thread: Thread started");
    
    // Benchmark start time
    benchmarkStartTime_ = millis();
    
    // Runtime parameters
    const uint32_t MIN_PROCESS_INTERVAL_MS = 5;  // Limit maximum rate to 200 Hz
    const uint32_t STATS_INTERVAL_MS = 30000;    // Print stats every 30 seconds
    
    uint32_t lastProcessTime = 0;
    uint32_t lastStatsTime = 0;
    
    // Main thread loop
    while (running_) {
        uint32_t currentTime = millis();
        
        // Process a batch if enough time has elapsed
        if (currentTime - lastProcessTime >= MIN_PROCESS_INTERVAL_MS) {
            // Start timing
            uint32_t startTime = micros();
            
            // Process and send a batch of samples
            size_t samplesProcessed = handler_->processAndSendBatch();
            
            if (samplesProcessed > 0) {
                // Record timing for benchmark
                uint32_t processingTime = micros() - startTime;
                recordProcessingTime(processingTime);
                
                // Update last process time
                lastProcessTime = currentTime;
                
                // Detailed log only for large batches to reduce spam
                if (samplesProcessed > 10) {
                    util::Debug::detail("PBUDP Thread: Processed " + String(samplesProcessed) + 
                                     " samples in " + String(processingTime) + "µs");
                }
            }
        }
        
        // Print benchmarks periodically
        if (currentTime - lastStatsTime >= STATS_INTERVAL_MS) {
            lastStatsTime = currentTime;
            printBenchmarks();
        }
        
        // Small yield to prevent thread hogging
        threads.yield();
    }
    
    // Print final benchmarks
    printBenchmarks();
    
    util::Debug::info("PBUDP Thread: Thread ended");
}

} // namespace network
} // namespace baja