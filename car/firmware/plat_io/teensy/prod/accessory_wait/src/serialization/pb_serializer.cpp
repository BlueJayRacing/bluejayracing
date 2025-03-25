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

// Helper function: hard-coded encoding for FixedDataChunk submessage.
// Optimized hard-coded encoding for FixedDataChunk submessage.
static bool encodeFixedDataChunk(pb_ostream_t *stream, const data::ChannelSample *samples, size_t count) {
    Serial.print("Encoding FixedDataChunk with " + String(count) + " samples\n");
    uint32_t start_time = micros();

    // Field 1: Encode base_timestamp (fixed64) from the first sample.
    uint64_t base_timestamp = samples[0].timestamp;
    if (!pb_encode_tag(stream, PB_WT_64BIT, FixedDataChunk_base_timestamp_tag))
        return false;
    if (!pb_encode_fixed64(stream, &base_timestamp))
        return false;

    // We'll use the same temporary buffer for each sample.
    uint8_t sample_buffer[32];
    // Cache the output pointer to avoid repeated pointer arithmetic.
    uint8_t *out_buffer = (uint8_t *)stream->state;
    
    size_t valid_samples = count;
    for (size_t i = 0; i < valid_samples; i++) {
        uint64_t delta = samples[i].timestamp - base_timestamp;
        if (delta > UINT16_MAX) {
            // If delta exceeds the maximum representable value,
            // truncate the batch (if not first sample) or clamp delta.
            if (i > 0) {
                valid_samples = i;
                break;
            } else {
                delta = UINT16_MAX;
            }
        }

        // Instead of creating a new temporary buffer each iteration,
        // reuse sample_buffer and create a temporary stream over it.
        pb_ostream_t sample_stream = pb_ostream_from_buffer(sample_buffer, sizeof(sample_buffer));

        // Encode Sample.field1: channel_id (varint)
        if (!pb_encode_tag(&sample_stream, PB_WT_VARINT, Sample_channel_id_tag))
            return false;
        if (!pb_encode_varint(&sample_stream, samples[i].channelIndex))
            return false;

        // Encode Sample.field2: value (varint)
        if (!pb_encode_tag(&sample_stream, PB_WT_VARINT, Sample_value_tag))
            return false;
        if (!pb_encode_varint(&sample_stream, samples[i].rawValue))
            return false;

        // Encode Sample.field3: timestamp_delta (varint)
        if (!pb_encode_tag(&sample_stream, PB_WT_VARINT, Sample_timestamp_delta_tag))
            return false;
        if (!pb_encode_varint(&sample_stream, delta))
            return false;

        // Now encode this Sample as a length-delimited field in the main stream.
        if (!pb_encode_tag(stream, PB_WT_STRING, FixedDataChunk_samples_tag))
            return false;
        if (!pb_encode_varint(stream, sample_stream.bytes_written))
            return false;
        if (stream->bytes_written + sample_stream.bytes_written > stream->max_size)
            return false;
        memcpy(&out_buffer[stream->bytes_written], sample_buffer, sample_stream.bytes_written);
        stream->bytes_written += sample_stream.bytes_written;
    }

    // Field 3: Encode sample_count.
    if (!pb_encode_tag(stream, PB_WT_VARINT, FixedDataChunk_sample_count_tag))
        return false;
    if (!pb_encode_varint(stream, valid_samples))
        return false;
    
    uint32_t end_time = micros();
    Serial.print("Encoding time: " + String(end_time - start_time) + "us\n");
    return true;
}


// Helper: compute how many bytes a varint will take.
static inline size_t varint_size(uint32_t value) {
    size_t size = 1;
    while (value >= 128) {
        value >>= 7;
        size++;
    }
    return size;
}

// Optimized hard-coded encoding for VerboseDataChunk submessage.
// The proto:
// message VerboseDataChunk {
//   repeated fixed64 timestamps = 1 [(nanopb).max_count = 100, (nanopb).fixed_count = true];
//   repeated uint32 channel_ids = 2 [(nanopb).max_count = 100, (nanopb).fixed_count = true, (nanopb).int_size = IS_8];
//   repeated uint32 values = 3 [(nanopb).max_count = 100, (nanopb).fixed_count = true];
//   uint32 sample_count = 4 [(nanopb).int_size = IS_16];
// }
static bool encodeVerboseDataChunk(pb_ostream_t *stream, const data::ChannelSample *samples, size_t count) {

    // 1. Encode timestamps as a packed fixed64 field.
    if (!pb_encode_tag(stream, PB_WT_STRING, VerboseDataChunk_timestamps_tag)) {
         Serial.print("Failed to encode timestamps tag\n");
         return false;
    }
    size_t timestamps_length = count * 8; // each fixed64 is 8 bytes
    if (!pb_encode_varint(stream, timestamps_length)) {
         Serial.print("Failed to encode timestamps length\n");
         return false;
    }
    for (size_t i = 0; i < count; i++) {
         if (!pb_encode_fixed64(stream, (uint64_t *)&samples[i].timestamp)) {
              Serial.print("Failed to encode timestamp at index " + String(i) + "\n");
              return false;
         }
    }

    // 2. Encode channel_ids as a packed varint field.
    if (!pb_encode_tag(stream, PB_WT_STRING, VerboseDataChunk_channel_ids_tag)) {
         Serial.print("Failed to encode channel_ids tag\n");
         return false;
    }
    size_t channel_ids_length = 0;
    for (size_t i = 0; i < count; i++) {
         channel_ids_length += varint_size(samples[i].channelIndex);
    }
    if (!pb_encode_varint(stream, channel_ids_length)) {
         Serial.print("Failed to encode channel_ids length\n");
         return false;
    }
    for (size_t i = 0; i < count; i++) {
         if (!pb_encode_varint(stream, samples[i].channelIndex)) {
              Serial.print("Failed to encode channel_id at index " + String(i) + "\n");
              return false;
         }
    }

    // 3. Encode values as a packed varint field.
    if (!pb_encode_tag(stream, PB_WT_STRING, VerboseDataChunk_values_tag)) {
         Serial.print("Failed to encode values tag\n");
         return false;
    }
    size_t values_length = 0;
    for (size_t i = 0; i < count; i++) {
         values_length += varint_size(samples[i].rawValue);
    }
    if (!pb_encode_varint(stream, values_length)) {
         Serial.print("Failed to encode values length\n");
         return false;
    }
    for (size_t i = 0; i < count; i++) {
         if (!pb_encode_varint(stream, samples[i].rawValue)) {
              Serial.print("Failed to encode value at index " + String(i) + "\n");
              return false;
         }
    }

    // 4. Encode sample_count (non-packed, varint).
    if (!pb_encode_tag(stream, PB_WT_VARINT, VerboseDataChunk_sample_count_tag)) {
         Serial.print("Failed to encode sample_count tag\n");
         return false;
    }
    if (!pb_encode_varint(stream, count)) {
         Serial.print("Failed to encode sample_count\n");
         return false;
    }
    
    return true;
}


// Refactored PBSerializer::encodeSamples() using hard-coded encoding.
static bool hardEncodeSamples(
    const data::ChannelSample* samples, 
    size_t count, 
    EncodedMessage& encodedMsg)
{
    uint32_t start_time = micros();
    if (count == 0 || count > 1000) {
        util::Debug::error("PBSerializer: Invalid sample count: " + String(count));
        return false;
    }
    
    // First pass: encode the submessage (either FixedDataChunk or VerboseDataChunk)
    uint8_t submsg_buffer[config::PB_MAX_MESSAGE_SIZE];
    pb_ostream_t submsg_stream = pb_ostream_from_buffer(submsg_buffer, sizeof(submsg_buffer));
    bool status = false;
    if (config::USE_VERBOSE_DATA_CHUNK) {
        status = encodeVerboseDataChunk(&submsg_stream, samples, count);
    } else {
        status = encodeFixedDataChunk(&submsg_stream, samples, count);
    }
    if (!status) {
        util::Debug::error("PBSerializer: Failed to hard-code encode samples");
        return false;
    }
    
    // Second pass: encode the wrapper DataChunk message.
    // In our .proto, DataChunk is defined as a oneof with:
    //    FixedDataChunk fixed = 1;
    //    VerboseDataChunk verbose = 2;
    // We'll choose the appropriate one based on our configuration.
    
    pb_ostream_t main_stream = pb_ostream_from_buffer(encodedMsg.buffer, config::PB_MAX_MESSAGE_SIZE);
    
    // Determine the oneof field tag to use.
    uint32_t oneof_field = config::USE_VERBOSE_DATA_CHUNK ? DataChunk_verbose_tag : DataChunk_fixed_tag;
    // For oneof fields, the wire type is length-delimited.
    if (!pb_encode_tag(&main_stream, PB_WT_STRING, oneof_field)) {
        util::Debug::error("PBSerializer: Failed to encode oneof tag");
        return false;
    }
    
    // Next, encode the length of the submessage.
    if (!pb_encode_varint(&main_stream, submsg_stream.bytes_written)) {
        util::Debug::error("PBSerializer: Failed to encode submessage length");
        return false;
    }
    
    // Finally, copy the pre-encoded submessage into the main stream.
    if (main_stream.bytes_written + submsg_stream.bytes_written > config::PB_MAX_MESSAGE_SIZE) {
        util::Debug::error("PBSerializer: Main buffer overflow");
        return false;
    }
    memcpy(&encodedMsg.buffer[main_stream.bytes_written], submsg_buffer, submsg_stream.bytes_written);
    main_stream.bytes_written += submsg_stream.bytes_written;

    encodedMsg.size = main_stream.bytes_written;
    encodedMsg.timestamp = millis();

    return true;
}

bool PBSerializer::encodeSamples(
    const data::ChannelSample* samples, 
    size_t count, 
    EncodedMessage& encodedMsg) {
    if (config::USE_HARD_CODED_ENCODING) {
        return hardEncodeSamples(samples, count, encodedMsg);
    }
    
    if (count == 0 || count > 1000) {
        util::Debug::error("PBSerializer: Invalid sample count: " + String(count));
        return false;
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
        
    } else {
        util::Debug::error("PBSerializer: Test encoding failed");
    }
    
    return success;
}

} // namespace serialization
} // namespace baja