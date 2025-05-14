#include "data_buffer.hpp"
#include <Arduino.h>

namespace baja {
namespace data {

DataBuffer::DataBuffer(size_t size)
    : buffer_(nullptr)
    , totalCapacity_(0)
    , segmentCapacity_(0)
    , currentWriteSegment_(0)
    , currentReadSegment_(0)
    , totalSamples_(0)
    , sdWriteTriggered_(false)
    , downsamplingRatio_(1)  // Default: no downsampling
{
    initializeBuffer(size);
}

DataBuffer::~DataBuffer() {
    // Free allocated memory based on how it was allocated
    if (buffer_) {
        // Since we used extmem_malloc, use extmem_free
        if (buffer_ != nullptr) {
            extmem_free(buffer_);
        } else {
            delete[] buffer_;
        }
        buffer_ = nullptr;
    }
}

bool DataBuffer::push(const ChannelSample& sample) {
    size_t writeSegment = currentWriteSegment_;
    
    // Lock the current write segment
    Threads::Scope lock(segments_[writeSegment].mutex);
    
    if (segments_[writeSegment].status != SegmentStatus::FILLING && 
        segments_[writeSegment].status != SegmentStatus::EMPTY) {
        // Current segment is not available for writing
        // Try to find another segment
        int newSegment = findSegmentWithStatus(SegmentStatus::EMPTY);
        if (newSegment < 0) {
            return false; // No empty segments available
        }
        
        // Switch to the new segment
        Threads::Scope globalLock(globalMutex_);
        currentWriteSegment_ = newSegment;
        writeSegment = newSegment;
        
        // Lock the new segment
        Threads::Scope newLock(segments_[writeSegment].mutex);
        segments_[writeSegment].status = SegmentStatus::FILLING;
    }
    
    // Check if the segment has space
    if (segments_[writeSegment].count >= segments_[writeSegment].capacity) {
        // Segment is full, try to move to next segment
        size_t nextSegment = nextSegmentIndex(writeSegment);
        
        // Try to lock the next segment
        if (segments_[nextSegment].mutex.try_lock()) {
            // If next segment is empty, switch to it
            if (segments_[nextSegment].status == SegmentStatus::EMPTY) {
                // Mark current segment as full
                segments_[writeSegment].status = SegmentStatus::FULL;
                
                // Switch to new segment
                Threads::Scope globalLock(globalMutex_);
                currentWriteSegment_ = nextSegment;
                segments_[nextSegment].status = SegmentStatus::FILLING;
                segments_[nextSegment].count = 0;
                
                // Unlock the next segment
                segments_[nextSegment].mutex.unlock();
                
                // Check if we should trigger SD write
                if (shouldTriggerSDWrite()) {
                    Threads::Scope globalLock(globalMutex_);
                    sdWriteTriggered_ = true;
                }
                
                // Try again with the new segment
                return push(sample);
            } else {
                // Next segment is not available
                segments_[nextSegment].mutex.unlock();
                return false;
            }
        } else {
            // Couldn't lock next segment
            return false;
        }
    }
    
    // Write the sample to the segment
    size_t index = segments_[writeSegment].count;
    segments_[writeSegment].samples[index] = sample;
    segments_[writeSegment].count++;
    
    // If this is the first sample in an empty segment, mark it as filling
    if (segments_[writeSegment].status == SegmentStatus::EMPTY) {
        segments_[writeSegment].status = SegmentStatus::FILLING;
    }
    
    // Update total samples count
    {
        Threads::Scope globalLock(globalMutex_);
        totalSamples_++;
    }
    
    // Check if we should trigger SD write after this addition
    if (shouldTriggerSDWrite() && !sdWriteTriggered_) {
        Threads::Scope globalLock(globalMutex_);
        sdWriteTriggered_ = true;
    }
    
    return true;
}

int DataBuffer::pushBatch(const ChannelSample* samples, size_t count) {
    size_t remaining = count;
    size_t pushed = 0;
    
    while (remaining > 0) {
        size_t writeSegment = currentWriteSegment_;
        
        // Lock the current write segment
        Threads::Scope lock(segments_[writeSegment].mutex);
        
        if (segments_[writeSegment].status != SegmentStatus::FILLING && 
            segments_[writeSegment].status != SegmentStatus::EMPTY) {
            // Current segment is not available for writing
            // Try to find another segment
            int newSegment = findSegmentWithStatus(SegmentStatus::EMPTY);
            if (newSegment < 0) {
                break; // No empty segments available
            }
            
            // Switch to the new segment
            Threads::Scope globalLock(globalMutex_);
            currentWriteSegment_ = newSegment;
            writeSegment = newSegment;
            
            // Update segment status
            segments_[writeSegment].status = SegmentStatus::FILLING;
            segments_[writeSegment].count = 0;
        }
        
        // Calculate how many samples we can push to this segment
        size_t available = segments_[writeSegment].capacity - segments_[writeSegment].count;
        size_t batchSize = (remaining < available) ? remaining : available;
        
        if (batchSize == 0) {
            // Segment is full, try to move to next segment
            size_t nextSegment = nextSegmentIndex(writeSegment);
            
            // Try to lock the next segment
            if (segments_[nextSegment].mutex.try_lock()) {
                // If next segment is empty, switch to it
                if (segments_[nextSegment].status == SegmentStatus::EMPTY) {
                    // Mark current segment as full
                    segments_[writeSegment].status = SegmentStatus::FULL;
                    
                    // Switch to new segment
                    Threads::Scope globalLock(globalMutex_);
                    currentWriteSegment_ = nextSegment;
                    segments_[nextSegment].status = SegmentStatus::FILLING;
                    segments_[nextSegment].count = 0;
                    
                    // Unlock the next segment
                    segments_[nextSegment].mutex.unlock();
                    
                    // Check if we should trigger SD write
                    if (shouldTriggerSDWrite()) {
                        Threads::Scope globalLock(globalMutex_);
                        sdWriteTriggered_ = true;
                    }
                    
                    // Continue with the new segment in the next iteration
                    continue;
                } else {
                    // Next segment is not available
                    segments_[nextSegment].mutex.unlock();
                    break;
                }
            } else {
                // Couldn't lock next segment
                break;
            }
        }
        
        // Copy the samples to the segment
        size_t startIndex = segments_[writeSegment].count;
        memcpy(&segments_[writeSegment].samples[startIndex], 
               &samples[pushed], 
               batchSize * sizeof(ChannelSample));
        
        segments_[writeSegment].count += batchSize;
        pushed += batchSize;
        remaining -= batchSize;
        
        // If this is the first batch in an empty segment, mark it as filling
        if (segments_[writeSegment].status == SegmentStatus::EMPTY) {
            segments_[writeSegment].status = SegmentStatus::FILLING;
        }
        
        // Update total samples count
        {
            Threads::Scope globalLock(globalMutex_);
            totalSamples_ += batchSize;
        }
        
        // Check if segment is now full
        if (segments_[writeSegment].count >= segments_[writeSegment].capacity) {
            segments_[writeSegment].status = SegmentStatus::FULL;
        }
        
        // Check if we should trigger SD write
        if (shouldTriggerSDWrite() && !sdWriteTriggered_) {
            Threads::Scope globalLock(globalMutex_);
            sdWriteTriggered_ = true;
        }
    }
    
    return pushed;
}

bool DataBuffer::shouldTriggerSDWrite() const {
    // Calculate how many segments are full or being filled
    int fullSegments = 0;
    
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (segments_[i].status == SegmentStatus::FULL || 
            segments_[i].status == SegmentStatus::FILLING) {
            fullSegments++;
        }
    }
    
    // Trigger SD write if the threshold is reached
    return static_cast<float>(fullSegments) / NUM_SEGMENTS >= SD_WRITE_THRESHOLD;
}

std::pair<const ChannelSample*, size_t> DataBuffer::beginSDWrite() {
    // Don't start a new write if one is already in progress
    if (currentReadSegment_ != 0 && 
        segments_[currentReadSegment_].status == SegmentStatus::READING) {
        return std::make_pair(nullptr, 0);
    }
    
    // Find a full segment to read from
    int segment = findSegmentWithStatus(SegmentStatus::FULL);
    if (segment < 0) {
        return std::make_pair(nullptr, 0);
    }
    
    // Lock the segment
    Threads::Scope lock(segments_[segment].mutex);
    
    // Mark segment as being read
    segments_[segment].status = SegmentStatus::READING;
    
    // Update current read segment
    Threads::Scope globalLock(globalMutex_);
    currentReadSegment_ = segment;
    
    // Return pointer to segment data and its size
    return std::make_pair(segments_[segment].samples, segments_[segment].count);
}

void DataBuffer::completeSDWrite(bool success) {
    size_t segment = currentReadSegment_;
    
    // Lock the segment
    Threads::Scope lock(segments_[segment].mutex);
    
    if (segments_[segment].status == SegmentStatus::READING) {
        if (success) {
            // Mark segment as ready for MQTT
            segments_[segment].status = SegmentStatus::READY_FOR_MQTT;
        } else {
            // If write failed, keep the segment as FULL to retry
            segments_[segment].status = SegmentStatus::FULL;
        }
    }
    
    // Reset current read segment if this was it
    Threads::Scope globalLock(globalMutex_);
    if (currentReadSegment_ == segment) {
        currentReadSegment_ = 0;
    }
    
    // Reset SD write trigger if appropriate
    if (!shouldTriggerSDWrite()) {
        sdWriteTriggered_ = false;
    }
}

std::pair<size_t, size_t> DataBuffer::getDownsampledData(
    uint8_t ratio, 
    size_t lastSampleIndex, 
    ChannelSample* buffer, 
    size_t maxSamples) 
{
    if (ratio == 0) ratio = 1; // Prevent division by zero
    if (ratio > MAX_DOWNSAMPLE_RATIO) ratio = MAX_DOWNSAMPLE_RATIO;
    
    size_t samplesRetrieved = 0;
    size_t nextIndex = lastSampleIndex;
    
    // Only use segments that have been written to SD
    for (int segmentIdx = 0; segmentIdx < NUM_SEGMENTS; segmentIdx++) {
        if (segments_[segmentIdx].status != SegmentStatus::READY_FOR_MQTT) {
            continue;
        }
        
        // Try to lock this segment
        if (!segments_[segmentIdx].mutex.try_lock()) {
            continue; // Skip if can't lock
        }
        
        // Process the segment
        size_t segmentOffset = segmentIdx * segmentCapacity_;
        size_t startIdx = 0;
        
        // If this segment contains our last index, start from there
        if (lastSampleIndex >= segmentOffset && 
            lastSampleIndex < segmentOffset + segments_[segmentIdx].count) {
            startIdx = lastSampleIndex - segmentOffset;
        } else if (lastSampleIndex >= segmentOffset + segments_[segmentIdx].count) {
            // If we've processed this entire segment already, skip it
            segments_[segmentIdx].mutex.unlock();
            continue;
        }
        
        // Apply downsampling
        for (size_t i = startIdx; i < segments_[segmentIdx].count; i += ratio) {
            if (samplesRetrieved >= maxSamples) {
                break; // Buffer full
            }
            
            buffer[samplesRetrieved] = segments_[segmentIdx].samples[i];
            samplesRetrieved++;
            nextIndex = segmentOffset + i + 1;
        }
        
        segments_[segmentIdx].mutex.unlock();
        
        if (samplesRetrieved >= maxSamples) {
            break; // We've filled the buffer
        }
    }
    
    return std::make_pair(samplesRetrieved, nextIndex);
}

size_t DataBuffer::size() const {
    // Use a mutex to safely read the total samples
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return totalSamples_;
}

bool DataBuffer::isEmpty() const {
    // Use a mutex to safely read the total samples
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return totalSamples_ == 0;
}

bool DataBuffer::isFull() const {
    // Use a mutex to safely read the total samples
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return totalSamples_ >= totalCapacity_;
}

size_t DataBuffer::capacity() const {
    return totalCapacity_;
}

float DataBuffer::fillPercentage() const {
    // Use a mutex to safely read the total samples
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return static_cast<float>(totalSamples_) / totalCapacity_;
}

void DataBuffer::setDownsamplingRatio(uint8_t ratio) {
    if (ratio == 0) ratio = 1; // Prevent division by zero
    if (ratio > MAX_DOWNSAMPLE_RATIO) ratio = MAX_DOWNSAMPLE_RATIO;
    
    Threads::Scope lock(globalMutex_);
    downsamplingRatio_ = ratio;
}

uint8_t DataBuffer::getDownsamplingRatio() const {
    Threads::Scope lock(const_cast<Threads::Mutex&>(globalMutex_));
    return downsamplingRatio_;
}

// Private helper methods

size_t DataBuffer::nextSegmentIndex(size_t current) const {
    return (current + 1) % NUM_SEGMENTS;
}

int DataBuffer::findSegmentWithStatus(SegmentStatus status) const {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        // Use a non-const copy of the mutex to avoid const issues
        Threads::Mutex& mutex = const_cast<Threads::Mutex&>(segments_[i].mutex);
        
        // Use try_lock to avoid blocking
        if (mutex.try_lock()) {
            bool match = (segments_[i].status == status);
            mutex.unlock();
            
            if (match) {
                return i;
            }
        }
    }
    return -1; // Not found
}

size_t DataBuffer::calculateSegmentSize(size_t totalSize) const {
    // Calculate how many samples we can store in the buffer
    size_t sampleSize = sizeof(ChannelSample);
    size_t totalSamples = totalSize / sampleSize;
    
    // Divide by number of segments, ensuring even distribution
    return totalSamples / NUM_SEGMENTS;
}

void DataBuffer::initializeBuffer(size_t totalSize) {
    // Calculate segment size
    segmentCapacity_ = calculateSegmentSize(totalSize);
    totalCapacity_ = segmentCapacity_ * NUM_SEGMENTS;
    
    // Allocate the buffer using external memory if running on Teensy 4.1
    size_t bufferBytes = totalCapacity_ * sizeof(ChannelSample);
    
    // Use extmem_malloc for large buffers on Teensy 4.1
    void* rawBuffer = extmem_malloc(bufferBytes);
    
    // Check if memory allocation succeeded
    if (!rawBuffer) {
        Serial.println("ERROR: Failed to allocate memory for data buffer with extmem_malloc!");
        
        // Fall back to regular heap memory
        Serial.println("Falling back to regular heap memory (may not be optimal)");
        buffer_ = new ChannelSample[totalCapacity_];
        
        if (!buffer_) {
            Serial.println("CRITICAL ERROR: Failed to allocate memory for data buffer!");
            // We don't have a buffer, so set capacities to 0 to prevent crashes
            segmentCapacity_ = 0;
            totalCapacity_ = 0;
            return;
        }
    } else {
        // Cast void* to our buffer type
        buffer_ = static_cast<ChannelSample*>(rawBuffer);
    }
    
    // Log buffer info
    Serial.print("Data buffer initialized: ");
    Serial.print(totalCapacity_);
    Serial.print(" samples (");
    Serial.print(totalCapacity_ * sizeof(ChannelSample) / 1024);
    Serial.println(" KB)");
    
    // Initialize segments
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        segments_[i].samples = buffer_ + (i * segmentCapacity_);
        segments_[i].capacity = segmentCapacity_;
        segments_[i].count = 0;
        segments_[i].status = SegmentStatus::EMPTY;
    }
    
    // Set initial state
    currentWriteSegment_ = 0;
    currentReadSegment_ = 0;
    totalSamples_ = 0;
    sdWriteTriggered_ = false;
}

} // namespace data
} // namespace baja