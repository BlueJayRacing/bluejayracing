#include "serialization/pb_serializer.hpp"
#include "util/debug_util.hpp"
#include <string.h>

namespace baja {
namespace serialization {

PBSerializer::PBSerializer(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& inputBuffer,
                         SerializedBuffer& outputBuffer)
    : inputBuffer_(inputBuffer),
      outputBuffer_(outputBuffer),
      maxTimestampDelta_(65000),  // Default max delta (uint16 max - small safety margin)
      lastBaseTimestamp_(0),
      samplesProcessed_(0),
      fixedChunksCreated_(0),
      verboseChunksCreated_(0) {
    util::Debug::info("PBSerializer: Initialized");
}

PBSerializer::~PBSerializer() {
}

size_t PBSerializer::processSamples(size_t maxSamples) {
    // Check if there are samples available
    size_t availableSamples = inputBuffer_.available();
    if (availableSamples == 0) {
        return 0;
    }
    
    // Limit the number of samples to process
    size_t samplesToProcess = (availableSamples < maxSamples) ? availableSamples : maxSamples;
    
    // Create a temporary buffer for samples
    data::ChannelSample* samples = new data::ChannelSample[samplesToProcess];
    if (!samples) {
        util::Debug::error("PBSerializer: Failed to allocate sample buffer");
        return 0;
    }
    
    // Read samples into the buffer
    size_t samplesRead = inputBuffer_.readMultiple(samples, samplesToProcess);
    
    if (samplesRead == 0) {
        delete[] samples;
        return 0;
    }
    
    // Set base timestamp
    uint64_t baseTimestamp = samples[0].timestamp;
    
    // Check if we should use a fixed or verbose chunk
    bool useFixed = canUseFixedChunk(samples, samplesRead, baseTimestamp);
    
    // Create the appropriate data chunk
    bool success;
    if (useFixed) {
        success = createFixedDataChunk(samples, samplesRead, baseTimestamp);
        if (success) {
            fixedChunksCreated_++;
            util::Debug::detail("PBSerializer: Created fixed data chunk with " + String(samplesRead) + " samples");
        }
    } else {
        success = createVerboseDataChunk(samples, samplesRead);
        if (success) {
            verboseChunksCreated_++;
            util::Debug::detail("PBSerializer: Created verbose data chunk with " + String(samplesRead) + " samples");
        }
    }
    
    // Clean up
    delete[] samples;
    
    // Update counters
    if (success) {
        samplesProcessed_ += samplesRead;
        lastBaseTimestamp_ = baseTimestamp;
        return samplesRead;
    } else {
        util::Debug::error("PBSerializer: Failed to create data chunk");
        return 0;
    }
}

void PBSerializer::setMaxTimestampDelta(uint16_t maxDelta) {
    maxTimestampDelta_ = maxDelta;
    util::Debug::info("PBSerializer: Max timestamp delta set to " + String(maxDelta) + " microseconds");
}

String PBSerializer::getStats() const {
    String stats = "PBSerializer Stats:\n";
    stats += "  Samples processed: " + String(samplesProcessed_) + "\n";
    stats += "  Fixed chunks created: " + String(fixedChunksCreated_) + "\n";
    stats += "  Verbose chunks created: " + String(verboseChunksCreated_) + "\n";
    stats += "  Last base timestamp: " + String((uint32_t)(lastBaseTimestamp_ & 0xFFFFFFFF)) + "\n";
    stats += "  Buffer usage: " + String(outputBuffer_.bytesUsed()) + "/" + 
             String(SerializedBuffer::MAX_BUFFER_SIZE) + " bytes\n";
    stats += "  Messages in buffer: " + String(outputBuffer_.messageCount()) + "\n";
    
    return stats;
}

bool PBSerializer::createFixedDataChunk(const data::ChannelSample* samples, size_t count, uint64_t baseTimestamp) {
    // Create message structure
    FixedDataChunk fixedChunk = FixedDataChunk_init_zero;
    
    // Set base timestamp
    fixedChunk.base_timestamp = baseTimestamp;
    
    // Set sample count (limit to max size if needed)
    fixedChunk.sample_count = (count > 150) ? 150 : count;
    
    // Fill samples
    for (size_t i = 0; i < fixedChunk.sample_count; i++) {
        fixedChunk.samples[i].channel_id = samples[i].channelIndex;
        fixedChunk.samples[i].value = samples[i].rawValue;
        
        // Calculate timestamp delta
        uint64_t delta = samples[i].timestamp - baseTimestamp;
        if (delta > UINT16_MAX) {
            util::Debug::warning("PBSerializer: Timestamp delta too large: " + String((uint32_t)(delta & 0xFFFFFFFF)));
            delta = UINT16_MAX;
        }
        
        fixedChunk.samples[i].timestamp_delta = (uint16_t)delta;
    }
    
    // Create master chunk
    DataChunk dataChunk = DataChunk_init_zero;
    dataChunk.which_chunk_type = DataChunk_fixed_tag;
    dataChunk.chunk_type.fixed = fixedChunk;
    
    // Encode the message
    uint8_t buffer[SerializedBuffer::MAX_MESSAGE_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    if (!pb_encode(&stream, DataChunk_fields, &dataChunk)) {
        util::Debug::error("PBSerializer: Failed to encode fixed data chunk: " + String(PB_GET_ERROR(&stream)));
        return false;
    }
    
    // Add the message to the serialized buffer
    return outputBuffer_.addMessage(buffer, stream.bytes_written);
}

bool PBSerializer::createVerboseDataChunk(const data::ChannelSample* samples, size_t count) {
    // Create message structure
    VerboseDataChunk verboseChunk = VerboseDataChunk_init_zero;
    
    // Set sample count (limit to max size if needed)
    verboseChunk.sample_count = (count > 150) ? 150 : count;
    
    // Fill arrays
    for (size_t i = 0; i < verboseChunk.sample_count; i++) {
        verboseChunk.timestamps[i] = samples[i].timestamp;
        verboseChunk.channel_ids[i] = samples[i].channelIndex;
        verboseChunk.values[i] = samples[i].rawValue;
    }
    
    // Create master chunk
    DataChunk dataChunk = DataChunk_init_zero;
    dataChunk.which_chunk_type = DataChunk_verbose_tag;
    dataChunk.chunk_type.verbose = verboseChunk;
    
    // Encode the message
    uint8_t buffer[SerializedBuffer::MAX_MESSAGE_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    if (!pb_encode(&stream, DataChunk_fields, &dataChunk)) {
        util::Debug::error("PBSerializer: Failed to encode verbose data chunk: " + String(PB_GET_ERROR(&stream)));
        return false;
    }
    
    // Add the message to the serialized buffer
    return outputBuffer_.addMessage(buffer, stream.bytes_written);
}

bool PBSerializer::canUseFixedChunk(const data::ChannelSample* samples, size_t count, uint64_t baseTimestamp) {
    // Check if any sample has a timestamp delta that exceeds the maximum
    for (size_t i = 0; i < count; i++) {
        uint64_t delta = samples[i].timestamp - baseTimestamp;
        if (delta > maxTimestampDelta_) {
            util::Debug::detail("PBSerializer: Using verbose chunk due to large delta: " + 
                              String((uint32_t)(delta & 0xFFFFFFFF)) + " > " + 
                              String(maxTimestampDelta_));
            return false;
        }
    }
    
    return true;
}

// SerializedBuffer implementation
SerializedBuffer::SerializedBuffer()
    : bytesUsed_(0) {
    util::Debug::info("SerializedBuffer: Initialized with capacity " + String(MAX_BUFFER_SIZE) + " bytes");
}

SerializedBuffer::~SerializedBuffer() {
    clear();
}

bool SerializedBuffer::addMessage(const uint8_t* data, size_t size) {
    Threads::Scope lock(mutex_);
    
    // Check if the message is too large
    if (size > MAX_MESSAGE_SIZE) {
        util::Debug::error("SerializedBuffer: Message too large: " + String(size) + " bytes");
        return false;
    }
    
    // Check if we have space
    if (bytesUsed_ + size > MAX_BUFFER_SIZE) {
        util::Debug::warning("SerializedBuffer: Buffer full, message dropped");
        return false;
    }
    
    // Create a new message
    Message message;
    memcpy(message.data, data, size);
    message.size = size;
    
    // Add to the vector
    messages_.push_back(message);
    bytesUsed_ += size;
    
    return true;
}

size_t SerializedBuffer::getMessage(uint8_t* data, size_t maxSize) {
    Threads::Scope lock(mutex_);
    
    // Check if we have any messages
    if (messages_.empty()) {
        return 0;
    }
    
    // Get the oldest message
    const Message& message = messages_.front();
    
    // Check if the buffer is large enough
    if (message.size > maxSize) {
        util::Debug::error("SerializedBuffer: Output buffer too small: " + 
                         String(maxSize) + " < " + String(message.size));
        return 0;
    }
    
    // Copy the message to the output buffer
    memcpy(data, message.data, message.size);
    size_t size = message.size;
    
    // Remove the message from the buffer
    bytesUsed_ -= message.size;
    messages_.erase(messages_.begin());
    
    return size;
}

size_t SerializedBuffer::messageCount() const {
    Threads::Scope lock(mutex_);
    return messages_.size();
}

size_t SerializedBuffer::bytesUsed() const {
    Threads::Scope lock(mutex_);
    return bytesUsed_;
}

size_t SerializedBuffer::bytesFree() const {
    Threads::Scope lock(mutex_);
    return MAX_BUFFER_SIZE - bytesUsed_;
}

bool SerializedBuffer::isEmpty() const {
    Threads::Scope lock(mutex_);
    return messages_.empty();
}

bool SerializedBuffer::isFull() const {
    Threads::Scope lock(mutex_);
    return bytesUsed_ >= MAX_BUFFER_SIZE - MAX_MESSAGE_SIZE;
}

void SerializedBuffer::clear() {
    Threads::Scope lock(mutex_);
    messages_.clear();
    bytesUsed_ = 0;
}

} // namespace serialization
} // namespace baja