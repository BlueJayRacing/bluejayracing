#include "serialization/pb_serialization_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace serialization {

// Initialize static class members
PBSerializer* PBSerializationThread::serializer_ = nullptr;
SerializedBuffer* PBSerializationThread::serializedBuffer_ = nullptr;
int PBSerializationThread::threadId_ = -1;
volatile bool PBSerializationThread::running_ = false;
uint32_t PBSerializationThread::samplesProcessedTotal_ = 0;

bool PBSerializationThread::initialize(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer) {
    util::Debug::info("PB Serialization Thread: Initializing");
    
    // Create the serialized buffer
    serializedBuffer_ = new SerializedBuffer();
    if (!serializedBuffer_) {
        util::Debug::error("PB Serialization Thread: Failed to create serialized buffer");
        return false;
    }
    
    // Create the serializer
    serializer_ = new PBSerializer(ringBuffer, *serializedBuffer_);
    if (!serializer_) {
        util::Debug::error("PB Serialization Thread: Failed to create serializer");
        delete serializedBuffer_;
        serializedBuffer_ = nullptr;
        return false;
    }
    
    util::Debug::info("PB Serialization Thread: Initialization successful");
    return true;
}

int PBSerializationThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("PB Serialization Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Check if serializer is initialized
    if (!serializer_ || !serializedBuffer_) {
        util::Debug::error("PB Serialization Thread: Not initialized");
        return -1;
    }
    
    // Reset counters
    samplesProcessedTotal_ = 0;
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 8192);  // Larger stack for serialization
    
    if (threadId_ <= 0) {
        util::Debug::error("PB Serialization Thread: Failed to create thread");
        running_ = false;
        return -2;
    }
    
    util::Debug::info("PB Serialization Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool PBSerializationThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    threadId_ = -1;
    
    util::Debug::info("PB Serialization Thread: Stopped");
    return true;
}

bool PBSerializationThread::isRunning() {
    return running_ && threadId_ > 0;
}

SerializedBuffer* PBSerializationThread::getSerializedBuffer() {
    return serializedBuffer_;
}

String PBSerializationThread::getStats() {
    if (!serializer_) {
        return "PB Serialization Thread: Not initialized";
    }
    
    String stats = serializer_->getStats();
    stats += "  Total samples processed: " + String(samplesProcessedTotal_) + "\n";
    
    return stats;
}

void PBSerializationThread::setMaxTimestampDelta(uint16_t maxDelta) {
    if (serializer_) {
        serializer_->setMaxTimestampDelta(maxDelta);
    }
}

void PBSerializationThread::threadFunction(void* arg) {
    util::Debug::info("PB Serialization Thread: Thread started");
    
    // Set thread priority
    threads.setTimeSlice(threads.id(), 4);  // Medium-high priority
    
    // Wait for things to stabilize
    delay(1000);
    
    uint32_t lastStatsTime = 0;
    uint32_t samplesProcessed = 0;
    
    // Main thread loop
    while (running_) {
        // Process samples (max 150 at a time)
        size_t processedSamples = serializer_->processSamples(150);
        
        if (processedSamples > 0) {
            samplesProcessed += processedSamples;
            samplesProcessedTotal_ += processedSamples;
        }
        
        // Print stats every 30 seconds
        uint32_t currentTime = millis();
        if (currentTime - lastStatsTime > 30000) {
            lastStatsTime = currentTime;
            util::Debug::info("PB Serialization Thread: Processed " + String(samplesProcessed) + 
                           " samples in the last 30 seconds");
            samplesProcessed = 0;
            
            // Log detailed stats at debug level
            util::Debug::debug(serializer_->getStats());
        }
        
        // If no samples were processed, yield to other threads
        if (processedSamples == 0) {
            threads.yield();
            delay(1);  // Small delay to prevent tight loop
        }
    }
    
    util::Debug::info("PB Serialization Thread: Thread ended");
}

} // namespace serialization
} // namespace baja