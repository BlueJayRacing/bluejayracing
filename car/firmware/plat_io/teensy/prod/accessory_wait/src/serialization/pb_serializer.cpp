#include "serialization/pb_serializer.hpp"
#include "util/debug_util.hpp"
#include "pb_encode.h"
#include "baja_sample.pb.h"
#include <algorithm>

namespace baja {
namespace serialization {

// Initialize static class members
buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>* PBSerializer::sourceBuffer_ = nullptr;
buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>* PBSerializer::targetBuffer_ = nullptr;
size_t PBSerializer::lastReadPosition_ = 0;
uint32_t PBSerializer::encodedCount_ = 0;
uint32_t PBSerializer::sampleCount_ = 0;

bool PBSerializer::initialize(
    buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& sourceBuffer,
    buffer::RingBuffer<EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& targetBuffer) {
    
    sourceBuffer_ = &sourceBuffer;
    targetBuffer_ = &targetBuffer;
    
    lastReadPosition_ = 0;
    encodedCount_ = 0;
    sampleCount_ = 0;
    
    util::Debug::info("PBSerializer: Initialized");
    return true;
}

size_t PBSerializer::processBatch(size_t maxSamples) {
    if (!sourceBuffer_ || !targetBuffer_) {
        util::Debug::error("PBSerializer: Not initialized");
        return 0;
    }
    
    // Get number of available samples in the source buffer
    size_t availableSamples = sourceBuffer_->available();
    if (availableSamples == 0) {
        return 0;
    }
    
    // Check if target buffer has space
    if (targetBuffer_->free() < 1) {
        static uint32_t lastWarningTime = 0;
        uint32_t now = millis();
        if (now - lastWarningTime > 10000) {  // Only warn every 10 seconds
            util::Debug::warning("PBSerializer: Target buffer full (" + 
                               String(targetBuffer_->available()) + "/" + 
                               String(targetBuffer_->capacity()) + ")");
            lastWarningTime = now;
        }
        return 0;  // Skip processing if no space
    }
    
    // Limit to maximum number of samples per batch
    // This is constrained by the fixed size in the protobuf definition
    size_t samplesToProcess = std::min(std::min(availableSamples, maxSamples), 
                                      static_cast<size_t>(config::MAX_SAMPLES_PER_BATCH));
    
    // Create a temporary buffer for the samples we'll process
    data::ChannelSample* samples = new data::ChannelSample[samplesToProcess];
    if (!samples) {
        util::Debug::error("PBSerializer: Failed to allocate sample buffer");
        return 0;
    }
    
    // Read samples from the source ring buffer (using peek to not remove them yet)
    size_t actualSamples = 0;
    for (size_t i = 0; i < samplesToProcess; i++) {
        data::ChannelSample sample;
        if (sourceBuffer_->peek(sample, i + lastReadPosition_)) {
            samples[actualSamples++] = sample;
        }
    }
    
    size_t processedCount = 0;
    if (actualSamples > 0) {
        // Create and encode the message
        EncodedMessage encodedMsg;
        if (encodeSamples(samples, actualSamples, encodedMsg)) {
            // Add to target buffer
            if (targetBuffer_->write(encodedMsg)) {
                // Update statistics
                encodedCount_++;
                sampleCount_ += actualSamples;
                processedCount = actualSamples;
                
                // Update last read position
                lastReadPosition_ += actualSamples;
                
                util::Debug::detail("PBSerializer: Encoded " + String(actualSamples) + 
                                  " samples into a " + String(encodedMsg.size) + " byte message");
            } else {
                util::Debug::warning("PBSerializer: Target buffer full, encoded message discarded");
            }
        } else {
            util::Debug::error("PBSerializer: Failed to encode samples");
        }
    }
    
    // Free the temporary buffer
    delete[] samples;
    
    return processedCount;
}

bool PBSerializer::encodeSamples(
    const data::ChannelSample* samples, 
    size_t count, 
    EncodedMessage& encodedMsg) {
    
    if (count == 0 || count > 150) {
        util::Debug::error("PBSerializer: Invalid sample count: " + String(count));
        return false;
    }
    
    if (config::PB_DEBUG_LOGGING) {
        util::Debug::detail("PBSerializer: Encoding " + String(count) + 
                          " samples with " + String(config::USE_VERBOSE_DATA_CHUNK ? "verbose" : "fixed") + 
                          " chunk format");
    }
    
    // Create and initialize the DataChunk message
    DataChunk message = DataChunk_init_zero;
    
    if (config::USE_VERBOSE_DATA_CHUNK) {
        // Set up VerboseDataChunk
        message.which_chunk_type = DataChunk_verbose_tag;
        message.chunk_type.verbose.sample_count = count;
        
        // Add full timestamps and values for each sample
        for (size_t i = 0; i < count; i++) {
            const data::ChannelSample& sample = samples[i];
            
            // Add the sample data
            message.chunk_type.verbose.timestamps[i] = sample.timestamp;
            message.chunk_type.verbose.channel_ids[i] = sample.channelIndex;
            message.chunk_type.verbose.values[i] = sample.rawValue;
            
            // Log first few samples for debugging
            if (config::PB_DEBUG_LOGGING && i < 3) {
                util::Debug::detail("PBSerializer: Sample " + String(i) + 
                                  ": ch=" + String(sample.channelIndex) + 
                                  ", val=" + String(sample.rawValue) + 
                                  ", ts=" + String(sample.timestamp));
            }
        }
    } else {
        // Set up FixedDataChunk with differential timestamps
        message.which_chunk_type = DataChunk_fixed_tag;
        message.chunk_type.fixed.base_timestamp = samples[0].timestamp;
        message.chunk_type.fixed.sample_count = count;
        
        // Add samples with differential timestamps
        for (size_t i = 0; i < count; i++) {
            const data::ChannelSample& sample = samples[i];
            
            // Calculate timestamp delta from base timestamp
            uint64_t delta = sample.timestamp - message.chunk_type.fixed.base_timestamp;
            
            // Check if delta exceeds the maximum representable value in protobuf (uint16)
            if (delta > UINT16_MAX) {
                util::Debug::warning("PBSerializer: Timestamp delta too large: " + 
                                  String(static_cast<uint32_t>(delta)) + 
                                  " for sample " + String(i));
                
                // Split the batch at this point if not the first sample
                if (i > 0) {
                    message.chunk_type.fixed.sample_count = i;
                    util::Debug::warning("PBSerializer: Truncating batch to " + String(i) + " samples");
                    break;
                } else {
                    // Force using verbose format for this batch
                    util::Debug::warning("PBSerializer: First sample has large delta, switching to verbose format");
                    return encodeSamples(samples, count, encodedMsg);  // Retry with verbose format
                }
            }
            
            // Add the sample to the message
            Sample& pbSample = message.chunk_type.fixed.samples[i];
            pbSample.channel_id = sample.channelIndex;
            pbSample.value = sample.rawValue;
            pbSample.timestamp_delta = static_cast<uint16_t>(delta);
            
            // Log first few samples for debugging
            if (config::PB_DEBUG_LOGGING && i < 3) {
                util::Debug::detail("PBSerializer: Sample " + String(i) + 
                                  ": ch=" + String(sample.channelIndex) + 
                                  ", val=" + String(sample.rawValue) + 
                                  ", delta=" + String(delta));
            }
        }
    }
    
    // Create a stream that writes to the encodedMsg buffer
    pb_ostream_t stream = pb_ostream_from_buffer(encodedMsg.buffer, config::PB_MAX_MESSAGE_SIZE);
    
    // Encode the message
    bool status = pb_encode(&stream, DataChunk_fields, &message);
    
    if (status) {
        encodedMsg.size = stream.bytes_written;
        encodedMsg.timestamp = millis();
        
        if (config::PB_DEBUG_LOGGING) {
            // util::Debug::info("PBSerializer: Successfully encoded " + 
            //                String(count) + " samples into " + 
            //                String(stream.bytes_written) + " bytes");
            
            // Debug: Show first few bytes of encoded message
            String hexBytes = "";
            for (size_t i = 0; i < min(16UL, stream.bytes_written); i++) {
                char hex[3];
                sprintf(hex, "%02X", encodedMsg.buffer[i]);
                hexBytes += hex;
                hexBytes += " ";
            }
            util::Debug::detail("PBSerializer: First bytes: " + hexBytes);
        }
        return true;
    } else {
        util::Debug::error("PBSerializer: Encoding failed: " + String(PB_GET_ERROR(&stream)));
        return false;
    }
}

size_t PBSerializer::getLastReadPosition() {
    return lastReadPosition_;
}

void PBSerializer::resetLastReadPosition() {
    lastReadPosition_ = 0;
}

uint32_t PBSerializer::getEncodedCount() {
    return encodedCount_;
}

uint32_t PBSerializer::getSampleCount() {
    return sampleCount_;
}

bool PBSerializer::testEncoding() {
    // Create some test samples
    data::ChannelSample testSamples[5];
    uint64_t baseTime = micros();
    
    for (int i = 0; i < 5; i++) {
        testSamples[i].timestamp = baseTime + i * 1000; // 1ms intervals
        testSamples[i].channelIndex = i;
        testSamples[i].rawValue = 10000 + i * 1000;
    }
    
    // Try to encode them
    EncodedMessage encodedMsg;
    bool success = encodeSamples(testSamples, 5, encodedMsg);
    
    if (success) {
        util::Debug::info("PBSerializer: Test encoding succeeded - " + 
                      String(encodedMsg.size) + " bytes encoded");
        
        // Print the entire encoded message in hex
        String hexData = "";
        for (size_t i = 0; i < encodedMsg.size; i++) {
            char hex[3];
            sprintf(hex, "%02X", encodedMsg.buffer[i]);
            hexData += hex;
            
            // Add space every 2 bytes for readability
            if (i % 2 == 1) hexData += " ";
            
            // Add newline every 16 bytes
            if (i % 16 == 15) hexData += "\n";
        }
        util::Debug::detail("PBSerializer: Encoded data:\n" + hexData);
        
        // If we have a target buffer, also write the test message to it
        if (targetBuffer_) {
            if (targetBuffer_->write(encodedMsg)) {
                util::Debug::info("PBSerializer: Test message added to encoded buffer");
            } else {
                util::Debug::error("PBSerializer: Failed to add test message to buffer");
            }
        }
    } else {
        util::Debug::error("PBSerializer: Test encoding failed");
    }
    
    return success;
}

} // namespace serialization
} // namespace baja