#include "network/pbudp_handler.hpp"
#include "util/debug_util.hpp"
#include "pb_encode.h"
#include "baja_sample.pb.h"


#define LinkStatus_kLinkStatusUp 1
#define USING_DHCP true

namespace baja {
namespace network {

using namespace qindesign::network;

PBUDPHandler::PBUDPHandler(buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer)
    : sourceBuffer_(sourceBuffer),
      messagesSent_(0),
      sampleCount_(0),
      bytesTransferred_(0),
      sendErrors_(0),
      lastStatsTime_(0),
      fastBufferOverflowCount_(0),
      port_(8888),
      isConnected_(false) {
    
    // Initialize server address to empty string
    serverAddress_[0] = '\0';
    
    util::Debug::info("PBUDPHandler: Handler initialized");
}

PBUDPHandler::~PBUDPHandler() {
    // Close UDP connection
    udp_.close();
    util::Debug::info("PBUDPHandler: Handler destroyed");
}

bool PBUDPHandler::initialize(const char* serverAddress, uint16_t port) {
    // Store server configuration
    strncpy(serverAddress_, serverAddress, sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    port_ = port;
    
    util::Debug::info("PBUDPHandler: Initializing for " + String(serverAddress_) + ":" + String(port_));
    
    // Set up the QNEthernet connection if not already initialized
    if (!isConnected()) {
        if (!initializeNetwork()) {
            util::Debug::error("PBUDPHandler: Network initialization failed!");
            return false;
        }
    }
    
    // Parse the server IP address
    IPAddress serverIP;
    int a, b, c, d;
    if (sscanf(serverAddress_, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
        serverIP = IPAddress(a, b, c, d);
    } else {
        int result = Ethernet.hostByName(serverAddress_, serverIP);
        if (result <= 0) {
            util::Debug::error("PBUDPHandler: Failed to resolve hostname: " + String(serverAddress_));
            return false;
        }
    }
    
    // Connect to the UDP server
    if (udp_.connect(serverIP, port_)) {
        util::Debug::info("PBUDPHandler: Connected to UDP server " + String(serverAddress_) + ":" + String(port_));
        isConnected_ = true;
        
        // Set up callback for incoming packets if needed
        udp_.onPacket([](AsyncUDPPacket packet) {
            // In our case, we don't expect responses, but we could process them here
            util::Debug::detail("PBUDPHandler: Received UDP packet from " + 
                                String(packet.remoteIP()) + ":" + String(packet.remotePort()) +
                                ", length: " + String(packet.length()));
        });
        
        return true;
    } else {
        util::Debug::error("PBUDPHandler: Failed to connect to UDP server");
        isConnected_ = false;
        return false;
    }
}

bool PBUDPHandler::initializeNetwork() {
    util::Debug::info("PBUDPHandler: Initializing QNEthernet...");
    
    // Check if Ethernet is already initialized
    if (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp && Ethernet.localIP() != IPAddress(0, 0, 0, 0)) {
        util::Debug::info("PBUDPHandler: Network already initialized with IP: " + ipToString(Ethernet.localIP()));
        return true;
    }
    
    
    #if USING_DHCP
        util::Debug::info("PBUDPHandler: Using DHCP...");
        if (!Ethernet.begin()) {
            util::Debug::error("PBUDPHandler: DHCP configuration failed");
            return false;
        }
        
        // Wait for DHCP to complete
        if (!Ethernet.waitForLocalIP(10000)) {
            util::Debug::error("PBUDPHandler: DHCP timeout");
            return false;
        }
    #else
        util::Debug::info("PBUDPHandler: Using static IP...");
        
        // Set up static IP configuration from defines.h
        IPAddress myIP(192, 168, 20, 13);
        IPAddress myNetmask(255, 255, 255, 0);
        IPAddress myGW(192, 168, 20, 1);
        IPAddress mydnsServer(8, 8, 8, 8);
        
        Ethernet.begin(myIP, myNetmask, myGW);
        Ethernet.setDNSServerIP(mydnsServer);
    #endif
    
    // Wait for link to be established
    uint32_t startTime = millis();
    util::Debug::info("PBUDPHandler: Waiting for network link...");
    while (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp && millis() - startTime < 5000) {
        delay(100);
    }
    
    if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
        util::Debug::error("PBUDPHandler: Ethernet link not established");
        return false;
    }
    
    // Get and log the IP address
    IPAddress localIP = Ethernet.localIP();
    util::Debug::info("PBUDPHandler: Connected with IP: " + ipToString(localIP));
    
    return true;
}

String PBUDPHandler::ipToString(IPAddress ip) {
    return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

size_t PBUDPHandler::processAndSendBatch() {
    uint32_t startTime = micros();
    
    // First check if network is connected
    if (!checkConnection()) {
        static uint32_t lastConnectionWarning = 0;
        if (millis() - lastConnectionWarning > 5000) {
            util::Debug::warning("PBUDPHandler: Network connection is down");
            lastConnectionWarning = millis();
        }
        return 0;
    }
    
    // Get number of available samples in the source buffer
    size_t availableSamples = sourceBuffer_.available();
    
    // Update buffer overflow stats if needed
    uint64_t overflowCount = sourceBuffer_.getOverwriteCount();
    if (overflowCount > fastBufferOverflowCount_) {
        uint64_t newOverflows = overflowCount - fastBufferOverflowCount_;
        util::Debug::warning("PBUDPHandler: Fast buffer overflow occurred, lost " + 
                           String(newOverflows) + " samples");
        fastBufferOverflowCount_ = overflowCount;
    }
    
    // Only process if we have exactly FIXED_SAMPLE_COUNT samples
    // Otherwise, we'd need to zero-pad which would be inefficient
    if (availableSamples < config::FIXED_SAMPLE_COUNT) {
        // Not enough samples yet, skip this cycle
        return 0;
    }
    
    // Read exactly FIXED_SAMPLE_COUNT samples from the source buffer
    size_t actualSamples = sourceBuffer_.readMultiple(sampleBuffer_, config::FIXED_SAMPLE_COUNT);
    
    if (actualSamples != config::FIXED_SAMPLE_COUNT) {
        util::Debug::warning("PBUDPHandler: Expected to read " + String(config::FIXED_SAMPLE_COUNT) + 
                           " samples but got " + String(actualSamples));
        return 0;
    }
    
    // Directly encode the samples to the encoded buffer
    size_t encodedSize = 0;
    bool encodingSuccess = encodeSamples(sampleBuffer_, actualSamples, encodedBuffer_, encodedSize);

    if (!encodingSuccess) {
        util::Debug::warning("PBUDPHandler: Failed to encode samples");
        return 0;
    }
    
    // Send the encoded message via UDP
    if (!sendEncodedData(encodedBuffer_, encodedSize)) {
        util::Debug::warning("PBUDPHandler: Failed to send encoded message");
        return 0;
    }
    
    // Update counters
    sampleCount_ += actualSamples;
    messagesSent_++;
    bytesTransferred_ += encodedSize;
    
    // Log statistics periodically
    logStats();
    
    uint32_t processingTime = micros() - startTime;
    util::Debug::detail("PBUDPHandler: Processed and sent " + String(actualSamples) + 
                       " samples in " + String(processingTime) + "µs");
    
    return actualSamples;
}

bool PBUDPHandler::encodeSamples(const data::ChannelSample* samples, size_t count, 
                               uint8_t* outputBuffer, size_t& outputSize) {
    if (count != config::FIXED_SAMPLE_COUNT) {
        util::Debug::error("PBUDPHandler: Invalid sample count: " + String(count) + 
                         ", must be exactly " + String(config::FIXED_SAMPLE_COUNT));
        return false;
    }
    
    // Choose between fixed and verbose encoding formats
    if (config::USE_HARD_CODED_ENCODING) {
        // Use hard-coded encoders for maximum performance
        if (config::USE_VERBOSE_DATA_CHUNK) {
            return encodeVerboseDataChunk(outputBuffer, config::PB_MAX_MESSAGE_SIZE, samples, count, outputSize);
        } else {
            return encodeFixedDataChunk(outputBuffer, config::PB_MAX_MESSAGE_SIZE, samples, count, outputSize);
        }
    } else {
        // Create and initialize the DataChunk message
        DataChunk message = DataChunk_init_zero;
        
        if (config::USE_VERBOSE_DATA_CHUNK) {
            // Set up VerboseDataChunk
            message.which_chunk_type = DataChunk_verbose_tag;
            message.chunk_type.verbose.sample_count = count;
            
            // Add full timestamps and values for each sample
            for (size_t i = 0; i < count; i++) {
                const data::ChannelSample& sample = samples[i];
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
                    // Truncate batch if not the first sample
                    if (i > 0) {
                        message.chunk_type.fixed.sample_count = i;
                        break;
                    } else {
                        // Force using verbose format for this batch
                        return encodeSamples(samples, count, outputBuffer, outputSize);
                    }
                }
                
                // Add the sample to the message
                Sample& pbSample = message.chunk_type.fixed.samples[i];
                pbSample.channel_id = sample.channelIndex;
                pbSample.value = sample.rawValue;
                pbSample.timestamp_delta = static_cast<uint16_t>(delta);
            }
        }
        
        // Create a stream that writes to the output buffer
        pb_ostream_t stream = pb_ostream_from_buffer(outputBuffer, config::PB_MAX_MESSAGE_SIZE);
        
        // Encode the message
        bool status = pb_encode(&stream, DataChunk_fields, &message);
        
        if (status) {
            outputSize = stream.bytes_written;
            return true;
        } else {
            util::Debug::error("PBUDPHandler: Encoding failed: " + String(PB_GET_ERROR(&stream)));
            return false;
        }
    }
}

// Helper function: compute how many bytes a varint will take
static inline size_t varint_size(uint32_t value) {
    size_t size = 1;
    while (value >= 128) {
        value >>= 7;
        size++;
    }
    return size;
}

bool PBUDPHandler::encodeFixedDataChunk(uint8_t* buffer, size_t bufferSize,
                                    const data::ChannelSample* samples, size_t count,
                                    size_t& outputSize) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);
    
    // First, encode the DataChunk wrapper with the fixed field
    if (!pb_encode_tag(&stream, PB_WT_STRING, DataChunk_fixed_tag)) {
        return false;
    }
    
    // We need to encode the size of the submessage, but we don't know it yet
    // So we'll reserve space for it by writing a dummy value
    size_t sizePos = stream.bytes_written;
    uint32_t dummySize = 0;
    if (!pb_encode_varint(&stream, dummySize)) {
        return false;
    }
    size_t submsgStart = stream.bytes_written;
    
    // Field 1: Encode base_timestamp (fixed64) from the first sample
    uint64_t base_timestamp = samples[0].timestamp;
    if (!pb_encode_tag(&stream, PB_WT_64BIT, FixedDataChunk_base_timestamp_tag)) {
        return false;
    }
    if (!pb_encode_fixed64(&stream, &base_timestamp)) {
        return false;
    }
    
    // Cache the output pointer to avoid repeated pointer arithmetic
    uint8_t* out_buffer = buffer;
    
    // We'll use the same temporary buffer for each sample
    uint8_t sample_buffer[32];
    
    // Process all samples
    size_t valid_samples = count;
    for (size_t i = 0; i < valid_samples; i++) {
        uint64_t delta = samples[i].timestamp - base_timestamp;
        if (delta > UINT16_MAX) {
            // If delta exceeds the maximum representable value,
            // truncate the batch (if not first sample) or clamp delta
            if (i > 0) {
                valid_samples = i;
                break;
            } else {
                delta = UINT16_MAX;
            }
        }
        
        // Use sample_buffer to avoid new allocations
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
            
        // Now encode this Sample as a length-delimited field in the main stream
        if (!pb_encode_tag(&stream, PB_WT_STRING, FixedDataChunk_samples_tag))
            return false;
        if (!pb_encode_varint(&stream, sample_stream.bytes_written))
            return false;
        if (stream.bytes_written + sample_stream.bytes_written > stream.max_size)
            return false;
        memcpy(&out_buffer[stream.bytes_written], sample_buffer, sample_stream.bytes_written);
        stream.bytes_written += sample_stream.bytes_written;
    }
    
    // Field 3: Encode sample_count
    if (!pb_encode_tag(&stream, PB_WT_VARINT, FixedDataChunk_sample_count_tag))
        return false;
    if (!pb_encode_varint(&stream, valid_samples))
        return false;
        
    // Now go back and write the correct submessage size
    size_t submsgSize = stream.bytes_written - submsgStart;
    uint8_t sizeBytes[10];
    pb_ostream_t sizeStream = pb_ostream_from_buffer(sizeBytes, sizeof(sizeBytes));
    if (!pb_encode_varint(&sizeStream, submsgSize)) {
        return false;
    }
    
    // Move the rest of the message forward if necessary
    if (sizeStream.bytes_written != 1) {
        memmove(
            &out_buffer[sizePos + sizeStream.bytes_written], 
            &out_buffer[sizePos + 1], 
            stream.bytes_written - (sizePos + 1)
        );
        stream.bytes_written += (sizeStream.bytes_written - 1);
    }
    
    // Copy the size bytes
    memcpy(&out_buffer[sizePos], sizeBytes, sizeStream.bytes_written);
    
    // Set output size
    outputSize = stream.bytes_written;
    return true;
}

bool PBUDPHandler::encodeVerboseDataChunk(uint8_t* buffer, size_t bufferSize,
                                      const data::ChannelSample* samples, size_t count,
                                      size_t& outputSize) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);
    
    // First, encode the DataChunk wrapper with the verbose field
    if (!pb_encode_tag(&stream, PB_WT_STRING, DataChunk_verbose_tag)) {
        return false;
    }
    
    // We need to encode the size of the submessage, but we don't know it yet
    // So we'll reserve space for it by writing a dummy value
    size_t sizePos = stream.bytes_written;
    uint32_t dummySize = 0;
    if (!pb_encode_varint(&stream, dummySize)) {
        return false;
    }
    size_t submsgStart = stream.bytes_written;

    // 1. Encode timestamps as a packed fixed64 field
    if (!pb_encode_tag(&stream, PB_WT_STRING, VerboseDataChunk_timestamps_tag)) {
        return false;
    }
    size_t timestamps_length = count * 8; // each fixed64 is 8 bytes
    if (!pb_encode_varint(&stream, timestamps_length)) {
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        if (!pb_encode_fixed64(&stream, (uint64_t *)&samples[i].timestamp)) {
            return false;
        }
    }
    
    // 2. Encode channel_ids as a packed varint field
    if (!pb_encode_tag(&stream, PB_WT_STRING, VerboseDataChunk_channel_ids_tag)) {
        return false;
    }
    size_t channel_ids_length = 0;
    for (size_t i = 0; i < count; i++) {
        channel_ids_length += varint_size(samples[i].channelIndex);
    }
    if (!pb_encode_varint(&stream, channel_ids_length)) {
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        if (!pb_encode_varint(&stream, samples[i].channelIndex)) {
            return false;
        }
    }
    
    // 3. Encode values as a packed varint field
    if (!pb_encode_tag(&stream, PB_WT_STRING, VerboseDataChunk_values_tag)) {
        return false;
    }
    size_t values_length = 0;
    for (size_t i = 0; i < count; i++) {
        values_length += varint_size(samples[i].rawValue);
    }
    if (!pb_encode_varint(&stream, values_length)) {
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        if (!pb_encode_varint(&stream, samples[i].rawValue)) {
            return false;
        }
    }
    
    // 4. Encode sample_count (non-packed, varint)
    if (!pb_encode_tag(&stream, PB_WT_VARINT, VerboseDataChunk_sample_count_tag)) {
        return false;
    }
    if (!pb_encode_varint(&stream, count)) {
        return false;
    }
    
    // Now go back and write the correct submessage size
    size_t submsgSize = stream.bytes_written - submsgStart;
    uint8_t sizeBytes[10];
    pb_ostream_t sizeStream = pb_ostream_from_buffer(sizeBytes, sizeof(sizeBytes));
    if (!pb_encode_varint(&sizeStream, submsgSize)) {
        return false;
    }
    
    // Move the rest of the message forward if necessary
    if (sizeStream.bytes_written != 1) {
        memmove(
            &buffer[sizePos + sizeStream.bytes_written], 
            &buffer[sizePos + 1], 
            stream.bytes_written - (sizePos + 1)
        );
        stream.bytes_written += (sizeStream.bytes_written - 1);
    }
    
    // Copy the size bytes
    memcpy(&buffer[sizePos], sizeBytes, sizeStream.bytes_written);
    
    // Set output size
    outputSize = stream.bytes_written;
    return true;
}

bool PBUDPHandler::sendEncodedData(const uint8_t* data, size_t size) {
    // Check if message is too large for UDP
    if (size > MAX_UDP_PAYLOAD) {
        util::Debug::warning("PBUDPHandler: Message size (" + String(size) + 
                         " bytes) exceeds UDP payload limit (" + String(MAX_UDP_PAYLOAD) + " bytes)");
        sendErrors_++;
        return false;
    }
    
    // Send the raw encoded data directly
    if (!udp_.write(data, size)) {
        util::Debug::warning("PBUDPHandler: Failed to send UDP message");
        sendErrors_++;
        return false;
    }
    
    return true;
}

bool PBUDPHandler::checkConnection() const {
    // Check if Ethernet link is up
    bool linkUp = (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp);
    
    // Check if we have a valid IP address
    IPAddress ip = Ethernet.localIP();
    bool validIP = !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
    
    return linkUp && validIP;
}

void PBUDPHandler::logStats() {
    uint32_t currentTime = millis();
    
    // Log stats every 10 seconds
    if (currentTime - lastStatsTime_ >= 10000) {
        lastStatsTime_ = currentTime;
        
        float bytesKB = bytesTransferred_ / 1024.0f;
        float avgMsgSize = messagesSent_ > 0 ? (float)bytesTransferred_ / messagesSent_ : 0;
        float samplesPerMsg = messagesSent_ > 0 ? (float)sampleCount_ / messagesSent_ : 0;
        
        util::Debug::info("PBUDPHandler: Stats - Sent: " + String(messagesSent_) + 
                      " messages (" + String(bytesKB, 1) + " KB), Samples: " +
                      String(sampleCount_) + ", Avg: " + String(samplesPerMsg, 1) + 
                      " samples/msg, " + String(avgMsgSize, 1) + " bytes/msg");
        
        if (sendErrors_ > 0) {
            util::Debug::warning("PBUDPHandler: Encountered " + String(sendErrors_) + 
                             " send errors");
        }
        
        // Log buffer statistics
        util::Debug::detail("PBUDPHandler: Fast buffer - Available: " + 
                        String(sourceBuffer_.available()) + "/" + 
                        String(sourceBuffer_.capacity()) + 
                        ", Overflows: " + String(fastBufferOverflowCount_));
    }
}

bool PBUDPHandler::isConnected() const {
  return isConnected_ && checkConnection();
}

} // namespace network
} // namespace baja