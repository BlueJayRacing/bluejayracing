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

// Benchmarking metrics
uint32_t PBThread::batchProcessingTimes_[100] = {0};
uint8_t PBThread::timeIndex_ = 0;
uint32_t PBThread::maxProcessingTime_ = 0;
uint32_t PBThread::minProcessingTime_ = UINT32_MAX;
uint32_t PBThread::totalProcessingTime_ = 0;
uint32_t PBThread::benchmarkStartTime_ = 0;

bool PBThread::initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& sourceBuffer,
    buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& targetBuffer) {
    
    util::Debug::info("PB Thread: Initializing");
    
    // Initialize PB serializer
    if (!PBSerializer::initialize(sourceBuffer, targetBuffer)) {
        util::Debug::error("PB Thread: Failed to initialize serializer");
        return false;
    }
    
    // Reset counters and benchmark metrics
    batchesProcessed_ = 0;
    samplesProcessed_ = 0;
    benchmarkStartTime_ = millis();
    maxProcessingTime_ = 0;
    minProcessingTime_ = UINT32_MAX;
    totalProcessingTime_ = 0;
    timeIndex_ = 0;
    
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
    
    // Reset benchmark start time
    benchmarkStartTime_ = millis();
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 100000);
    
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
    
    // Print final benchmarks before fully stopping
    printBenchmarks();
    
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

void PBThread::getTimingStats(float& avgProcessingTime, uint32_t& minProcessingTime, 
                           uint32_t& maxProcessingTime, uint32_t& totalBatches) {
    // Calculate average processing time
    avgProcessingTime = batchesProcessed_ > 0 ? 
                       totalProcessingTime_ / (float)batchesProcessed_ : 0.0f;
    
    minProcessingTime = minProcessingTime_;
    maxProcessingTime = maxProcessingTime_;
    totalBatches = batchesProcessed_;
}

void PBThread::recordProcessingTime(uint32_t processingTime) {
    // Store time in circular buffer
    batchProcessingTimes_[timeIndex_] = processingTime;
    timeIndex_ = (timeIndex_ + 1) % 100;
    
    // Update stats
    totalProcessingTime_ += processingTime;
    if (processingTime > maxProcessingTime_) {
        maxProcessingTime_ = processingTime;
    }
    if (processingTime < minProcessingTime_) {
        minProcessingTime_ = processingTime;
    }
}

void PBThread::printBenchmarks() {
    // Calculate elapsed time since benchmark start
    uint32_t elapsed = millis() - benchmarkStartTime_;
    if (elapsed == 0) elapsed = 1; // Avoid division by zero
    
    // Get current stats
    uint32_t encodedCount, sampleCount;
    getStats(encodedCount, sampleCount);
    
    // Calculate messages per second
    float batchesPerSec = batchesProcessed_ * 1000.0f / elapsed;
    float samplesPerSec = samplesProcessed_ * 1000.0f / elapsed;
    
    // Calculate average processing time
    float avgProcessingTime = batchesProcessed_ > 0 ? 
                             totalProcessingTime_ / (float)batchesProcessed_ : 0.0f;
    
    // Calculate samples per batch
    float samplesPerBatch = batchesProcessed_ > 0 ? 
                           samplesProcessed_ / (float)batchesProcessed_ : 0.0f;
    
    // Print benchmark results
    util::Debug::info("--------- PB SERIALIZATION BENCHMARKS ---------");
    util::Debug::info("Batches processed: " + String(batchesProcessed_) + 
                    " (" + String(batchesPerSec, 2) + " batches/sec)");
    util::Debug::info("Samples processed: " + String(samplesProcessed_) + 
                    " (" + String(samplesPerSec, 2) + " samples/sec)");
    util::Debug::info("Avg samples per batch: " + String(samplesPerBatch, 2));
    
    if (batchesProcessed_ > 0) {
        util::Debug::info("Processing time: avg=" + String(avgProcessingTime, 2) + 
                        "µs, min=" + String(minProcessingTime_) + 
                        "µs, max=" + String(maxProcessingTime_) + "µs");
    }
    
    // Calculate encoder efficiency
    float encoderEfficiency = sampleCount > 0 ? 
                            encodedCount * 100.0f / sampleCount * config::MAX_SAMPLES_PER_BATCH : 0.0f;
    
    util::Debug::info("Serializer efficiency: " + String(encoderEfficiency, 2) + "%");
    util::Debug::info("----------------------------------------------");
}

void PBThread::threadFunction(void* arg) {
    util::Debug::info("PB Thread: Thread started");
    
    // Set thread priority
    threads.setTimeSlice(threads.id(), 7);  // low priority
    
    // Reset benchmark timer
    benchmarkStartTime_ = millis();
    
    // Wait for things to stabilize before starting serialization
    delay(1000);
    
    uint32_t lastStatsTime = 0;
    uint32_t lastBenchmarkTime = 0;
    
    // Main thread loop
    while (running_) {
        uint32_t currentTime = millis();
        
        // Process data every 50ms (or as configured)
        if (currentTime - lastProcessTime_ >= config::PB_SERIALIZATION_INTERVAL_MS) {
            lastProcessTime_ = currentTime;
            
            // Process a batch of samples with timing
            constexpr size_t batchSize = config::MAX_SAMPLES_PER_BATCH;
            
            // Start timing
            uint32_t startMicros = micros();
            
            // Process a batch of samples
            size_t processed = PBSerializer::processBatch(batchSize);
            
            // End timing and record
            uint32_t processingTime = micros() - startMicros;
            
            if (processed > 0) {
                // Record processing time for benchmarking
                recordProcessingTime(processingTime);
                
                batchesProcessed_++;
                samplesProcessed_ += processed;
                
                // Debug output when processing batches (only for significant batches)
                if (processed >= 10) {
                    util::Debug::detail("PB Thread: Processed batch of " + String(processed) + 
                                     " samples in " + String(processingTime) + "µs");
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
        
        // Print benchmarks periodically (every 30 seconds)
        if (currentTime - lastBenchmarkTime >= 30000) {
            lastBenchmarkTime = currentTime;
            printBenchmarks();
        }
        
        // Small yield to allow other threads to run
        threads.yield();
    }
    
    // Print final benchmarks before ending
    printBenchmarks();
    
    util::Debug::info("PB Thread: Thread ended");
}

} // namespace serialization
} // namespace baja