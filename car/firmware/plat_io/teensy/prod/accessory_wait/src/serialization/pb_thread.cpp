#include "serialization/pb_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace serialization {

// Initialize static class members
int PBThread::threadId_ = -1;
volatile bool PBThread::running_ = false;
uint32_t PBThread::lastProcessTime_ = 0;
uint32_t PBThread::batchesProcessed_ = 0;
uint32_t PBThread::samplesProcessed_ = 0;

bool PBThread::initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& sourceBuffer,
    buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& targetBuffer) {
    
    util::Debug::info("PB Thread: Initializing");
    
    // Initialize PB serializer
    if (!PBSerializer::initialize(sourceBuffer, targetBuffer)) {
        util::Debug::error("PB Thread: Failed to initialize serializer");
        return false;
    }
    
    // Reset counters
    batchesProcessed_ = 0;
    samplesProcessed_ = 0;
    
    util::Debug::info("PB Thread: Initialization successful");
    
    // If we're in debug mode, run a test encoding to validate the pipeline
    if (config::PB_DEBUG_LOGGING) {
        util::Debug::info("PB Thread: Running test encoding (Verbose=" + 
                       String(config::USE_VERBOSE_DATA_CHUNK ? "true" : "false") + ")");
        if (PBSerializer::testEncoding()) {
            util::Debug::info("PB Thread: Test encoding successful");
        } else {
            util::Debug::error("PB Thread: Test encoding failed!");
        }
    }
    
    return true;
}

int PBThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("PB Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 50000);
    
    if (threadId_ <= 0) {
        util::Debug::error("PB Thread: Failed to create thread");
        running_ = false;
        return -1;
    }
    
    util::Debug::info("PB Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool PBThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    threadId_ = -1;
    
    util::Debug::info("PB Thread: Stopped");
    return true;
}

bool PBThread::isRunning() {
    return running_ && threadId_ > 0;
}

PBSerializer* PBThread::getSerializer() {
    return nullptr;  // PBSerializer is static, no instance to return
}

void PBThread::getStats(uint32_t& encodedCount, uint32_t& sampleCount) {
    encodedCount = PBSerializer::getEncodedCount();
    sampleCount = PBSerializer::getSampleCount();
}

void PBThread::threadFunction(void* arg) {
    util::Debug::info("PB Thread: Thread started");
    
    // Set thread priority
    threads.setTimeSlice(threads.id(), 4);  // Medium-high priority
    
    // Wait for things to stabilize before starting serialization
    delay(1000);
    
    uint32_t lastStatsTime = 0;
    
    // Main thread loop
    while (running_) {
        uint32_t currentTime = millis();
        
        // Process data every 50ms
        if (currentTime - lastProcessTime_ >= config::PB_SERIALIZATION_INTERVAL_MS) {
            lastProcessTime_ = currentTime;
            
            // Process a batch of samples
            constexpr size_t batchSize = config::MAX_SAMPLES_PER_BATCH;
            size_t processed = PBSerializer::processBatch(batchSize);
            
            if (processed > 0) {
                batchesProcessed_++;
                samplesProcessed_ += processed;
                
                // Debug output when processing batches (only for significant batches)
                if (processed >= 10) {
                    util::Debug::detail("PB Thread: Processed batch of " + String(processed) + " samples");
                }
            }
        }
        
        // Print stats every 30 seconds
        if (currentTime - lastStatsTime >= 30000) {
            lastStatsTime = currentTime;
            
            uint32_t encodedCount, sampleCount;
            getStats(encodedCount, sampleCount);
            
            util::Debug::info("PB Thread: Stats - Encoded messages: " + 
                String(encodedCount) + ", Total samples: " + String(sampleCount));
        }
        
        // Small yield to allow other threads to run
        threads.yield();
        
        // Add a small delay to prevent tight loop
        delay(5);
    }
    
    util::Debug::info("PB Thread: Thread ended");
}

} // namespace serialization
} // namespace baja