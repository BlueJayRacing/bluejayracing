#include "adc_data_queue.hpp"
#include <Arduino.h>

// TCM memory attribute for Teensy 4.1
#if defined(__IMXRT1062__)
#define TCM_MEMORY_ATTR __attribute__((section(".tcm")))
#else
#define TCM_MEMORY_ATTR
#endif

namespace baja {
namespace adc {

// Constructor
AdcDataQueue::AdcDataQueue(size_t bufferSize)
    : bufferSize_(bufferSize)
    , readIndex_(0)
    , writeIndex_(0)
    , count_(0)
{
    // Allocate buffer - For Teensy 4.1, we could use external PSRAM
    // but for simplicity, we'll use regular heap memory for now
    buffer_ = new AdcSample[bufferSize];
    
    clear();
}

// Destructor
AdcDataQueue::~AdcDataQueue() {
    // Free memory
    delete[] buffer_;
}

// Push a single sample
bool AdcDataQueue::push(const AdcSample& sample) {
    Threads::Scope lock(mutex_);
    
    if (isFull()) {
        return false;
    }
    
    // Copy sample to buffer
    buffer_[writeIndex_] = sample;
    
    // Update write index
    writeIndex_ = (writeIndex_ + 1) % bufferSize_;
    
    // Increment count
    count_++;
    
    return true;
}

// Push multiple samples
int AdcDataQueue::pushBatch(const AdcSample* samples, size_t count) {
    Threads::Scope lock(mutex_);
    
    // Calculate how many samples we can actually push
    size_t spaceAvailable = bufferSize_ - count_;
    size_t samplesToPush = (count < spaceAvailable) ? count : spaceAvailable;
    
    // Push each sample
    for (size_t i = 0; i < samplesToPush; i++) {
        buffer_[writeIndex_] = samples[i];
        writeIndex_ = (writeIndex_ + 1) % bufferSize_;
    }
    
    // Update count
    count_ += samplesToPush;
    
    return samplesToPush;
}

// Pop a single sample
bool AdcDataQueue::pop(AdcSample& sample) {
    Threads::Scope lock(mutex_);
    
    if (isEmpty()) {
        return false;
    }
    
    // Copy sample from buffer
    sample = buffer_[readIndex_];
    
    // Update read index
    readIndex_ = (readIndex_ + 1) % bufferSize_;
    
    // Decrement count
    count_--;
    
    return true;
}

// Pop multiple samples
int AdcDataQueue::popBatch(AdcSample* samples, size_t maxCount) {
    Threads::Scope lock(mutex_);
    
    // Calculate how many samples we can actually pop
    size_t samplesToPop = (maxCount < count_) ? maxCount : count_;
    
    // Pop each sample
    for (size_t i = 0; i < samplesToPop; i++) {
        samples[i] = buffer_[readIndex_];
        readIndex_ = (readIndex_ + 1) % bufferSize_;
    }
    
    // Update count
    count_ -= samplesToPop;
    
    return samplesToPop;
}

// Get number of samples in queue
size_t AdcDataQueue::size() const {
    // No need for lock here as count_ is atomic
    return count_;
}

// Check if queue is empty
bool AdcDataQueue::isEmpty() const {
    return count_ == 0;
}

// Check if queue is full
bool AdcDataQueue::isFull() const {
    return count_ >= bufferSize_;
}

// Clear the queue
void AdcDataQueue::clear() {
    Threads::Scope lock(mutex_);
    
    readIndex_ = 0;
    writeIndex_ = 0;
    count_ = 0;
}

} // namespace adc
} // namespace baja