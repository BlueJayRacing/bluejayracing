#include "network/pbudp_handler.hpp"
#include "util/debug_util.hpp"
#include "pb_encode.h"
#include "teensy_data.pb.h"


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
    
    // Group counters for channel activity tracking
    std::array<uint32_t, util::TOTAL_CHANNEL_COUNT> channelCounts = {};
    
    // Count samples by channel ID type
    uint32_t adcCount = 0;
    uint32_t dinCount = 0;
    uint32_t miscCount = 0;
    
    // Analyze sample distribution for logging
    for (size_t i = 0; i < actualSamples; i++) {
        uint8_t chId = sampleBuffer_[i].internalChannelId;
        if (chId < util::TOTAL_CHANNEL_COUNT) {
            channelCounts[chId]++;
            
            // Track by channel type
            if (chId < 16) adcCount++;
            else if (chId < 22) dinCount++;
            else miscCount++;
        }
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
    
    // Log detailed stats periodically
    static uint32_t lastDetailLogTime = 0;
    uint32_t currentTime = millis();
    
    if (currentTime - lastDetailLogTime > 30000) { // Every 30 seconds
        lastDetailLogTime = currentTime;
        
        util::Debug::detail("PBUDPHandler: Channel distribution in last batch - ADC: " + String(adcCount) + ", " +
                          "DIN: " + String(dinCount) + ", " +
                          "MISC: " + String(miscCount));
    }
    
    // Log statistics periodically
    logStats();
    
    uint32_t processingTime = micros() - startTime;
    util::Debug::detail("PBUDPHandler: Processed and sent " + String(actualSamples) + 
                       " samples in " + String(processingTime) + "µs");
    
    return actualSamples;
}

bool PBUDPHandler::encodeSamples(const data::ChannelSample* samples, size_t count, 
        uint8_t* outputBuffer, size_t& outputSize) {
    if (count > config::FIXED_SAMPLE_COUNT) {
        util::Debug::error("PBUDPHandler: Invalid sample count: " + String(count) + 
            ", must be no more than " + String(config::FIXED_SAMPLE_COUNT));
        return false;
    }
    
    if (config::USE_HARD_CODED_ENCODING) {
        // Use hard-coded encoder for maximum performance.
        return encodeDataChunk(outputBuffer, config::PB_MAX_MESSAGE_SIZE, samples, count, outputSize);
    } else {
        // Create and initialize the DataChunk message
        DataChunk message = DataChunk_init_zero;
        message.sample_count = count;
        for (size_t i = 0; i < count; i++) {
            message.timestamps[i] = samples[i].timestamp;
            message.internal_channel_ids[i] = samples[i].internalChannelId;
            message.values[i] = samples[i].rawValue;
        }
        pb_ostream_t stream = pb_ostream_from_buffer(outputBuffer, config::PB_MAX_MESSAGE_SIZE);
        if (!pb_encode(&stream, DataChunk_fields, &message)) {
            util::Debug::error("PBUDPHandler: Encoding failed: " + String(PB_GET_ERROR(&stream)));
            return false;
        }
        outputSize = stream.bytes_written;
        return true;
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

bool PBUDPHandler::encodeDataChunk(uint8_t* buffer, size_t bufferSize,
        const data::ChannelSample* samples, size_t count,
        size_t& outputSize) {
    // Create a stream that writes directly into the output buffer.
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);

    // 1. Encode the timestamps field (packed fixed64)
    // Write the tag for a length-delimited field.
    if (!pb_encode_tag(&stream, PB_WT_STRING, DataChunk_timestamps_tag))
        return false;
    // Calculate and encode the byte length of the packed timestamps.
    size_t timestamps_length = count * 8; // each fixed64 is 8 bytes
    if (!pb_encode_varint(&stream, timestamps_length))
        return false;
    // Write each timestamp.
    for (size_t i = 0; i < count; i++) {
        if (!pb_encode_fixed64(&stream, (uint64_t *)&samples[i].timestamp))
            return false;
    }

    // 2. Encode the internal_channel_ids field (packed varint)
    if (!pb_encode_tag(&stream, PB_WT_STRING, DataChunk_internal_channel_ids_tag))
        return false;
    size_t channel_ids_length = 0;
    for (size_t i = 0; i < count; i++) {
        channel_ids_length += varint_size(samples[i].internalChannelId);
    }
    if (!pb_encode_varint(&stream, channel_ids_length))
        return false;
    for (size_t i = 0; i < count; i++) {
        if (!pb_encode_varint(&stream, samples[i].internalChannelId))
            return false;
    }

    // 3. Encode the values field (packed varint)
    if (!pb_encode_tag(&stream, PB_WT_STRING, DataChunk_values_tag))
        return false;
    size_t values_length = 0;
    for (size_t i = 0; i < count; i++) {
        values_length += varint_size(samples[i].rawValue);
    }
    if (!pb_encode_varint(&stream, values_length))
        return false;
    for (size_t i = 0; i < count; i++) {
        if (!pb_encode_varint(&stream, samples[i].rawValue))
            return false;
    }

    // 4. Encode the sample_count field (non-packed varint)
    if (!pb_encode_tag(&stream, PB_WT_VARINT, DataChunk_sample_count_tag))
        return false;
    if (!pb_encode_varint(&stream, count))
        return false;

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