#pragma once

#include <Arduino.h>
#include <QNEthernet.h>
#include <AsyncUDP_Teensy41.hpp>
#include "util/circular_buffer.hpp"
#include "util/sample_data.hpp"
#include "config/config.hpp"
#include "baja_sample.pb.h"

namespace baja {
namespace network {

/**
 * @brief Combined Protocol Buffer serializer and UDP sender class
 * 
 * This class efficiently processes samples directly from the fast path buffer,
 * encodes them into protobuf messages, and sends them via UDP without
 * intermediate buffers or thread switching.
 */
class PBUDPHandler {
public:
    /**
     * @brief Construct a new PBUDPHandler
     * 
     * @param sourceBuffer Reference to the fast path buffer of samples to process
     */
    PBUDPHandler(buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer);
    
    /**
     * @brief Destroy the PBUDPHandler
     */
    ~PBUDPHandler();
    
    /**
     * @brief Initialize UDP client with server address and port
     * 
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @return true if initialization was successful
     */
    bool initialize(const char* serverAddress, uint16_t port = 8888);
    
    /**
     * @brief Initialize network connection with proper error handling
     * 
     * @return true if network initialized successfully
     */
    bool initializeNetwork();
    
    /**
     * @brief Process and send a batch of samples
     * 
     * Efficiently processes exactly FIXED_SAMPLE_COUNT samples from the source buffer,
     * encodes them into a protobuf message, and sends directly via UDP.
     * If fewer than FIXED_SAMPLE_COUNT samples are available, no processing occurs.
     * 
     * @return Number of samples processed (either FIXED_SAMPLE_COUNT or 0)
     */
    size_t processAndSendBatch();
    
    /**
     * @brief Convert IP address to string for logging
     * 
     * @param ip IP address to convert
     * @return String representation of the IP address
     */
    static String ipToString(IPAddress ip);
    
    /**
     * @brief Check if connected to the network
     * 
     * @return true if connected
     */
    bool isConnected() const;
    
    /**
     * @brief Get the number of messages sent
     * 
     * @return Message count
     */
    uint32_t getMessagesSent() const { return messagesSent_; }
    
    /**
     * @brief Get the number of samples processed
     * 
     * @return Sample count
     */
    uint32_t getSampleCount() const { return sampleCount_; }
    
    /**
     * @brief Get the number of bytes transferred
     * 
     * @return Bytes sent
     */
    uint32_t getBytesTransferred() const { return bytesTransferred_; }
    
    /**
     * @brief Get the number of send errors
     * 
     * @return Error count
     */
    uint32_t getSendErrors() const { return sendErrors_; }

private:
    // Pre-allocated buffer for samples to process
    data::ChannelSample sampleBuffer_[config::FIXED_SAMPLE_COUNT];
    
    // Pre-allocated buffer for the encoded message
    uint8_t encodedBuffer_[config::PB_MAX_MESSAGE_SIZE];
    
    // Source buffer reference (fast path buffer)
    buffer::CircularBuffer<data::ChannelSample, config::FAST_BUFFER_SIZE>& sourceBuffer_;
    
    // UDP client
    AsyncUDP udp_;
    
    // Statistics
    uint32_t messagesSent_;
    uint32_t sampleCount_;
    uint32_t bytesTransferred_;
    uint32_t sendErrors_;
    uint32_t lastStatsTime_;
    uint32_t fastBufferOverflowCount_;
    
    // Server configuration
    char serverAddress_[64];
    uint16_t port_;
    bool isConnected_;
    
    // Maximum UDP payload size (typical Ethernet MTU minus headers)
    static const size_t MAX_UDP_PAYLOAD = 1472;
    
    /**
     * @brief Encode samples directly into output buffer without intermediate allocation
     * 
     * @param samples Array of samples to encode
     * @param count Number of samples to encode
     * @param outputBuffer Buffer to store encoded message
     * @param outputSize Output parameter for size of encoded message
     * @return true if encoding successful
     */
    bool encodeSamples(const data::ChannelSample* samples, size_t count, 
                      uint8_t* outputBuffer, size_t& outputSize);
    
    /**
     * @brief Send encoded data directly via UDP
     * 
     * @param data Pointer to encoded data
     * @param size Size of encoded data
     * @return true if send was successful
     */
    bool sendEncodedData(const uint8_t* data, size_t size);
    
    /**
     * @brief Check if the network is connected
     * 
     * @return true if connected
     */
    bool checkConnection() const;
    
    /**
     * @brief Log statistics periodically
     */
    void logStats();
    
    /**
     * @brief Helper: Hard-coded encoding for FixedDataChunk
     */
    bool encodeFixedDataChunk(uint8_t* buffer, size_t bufferSize, 
                           const data::ChannelSample* samples, size_t count,
                           size_t& outputSize);
    
    /**
     * @brief Helper: Hard-coded encoding for VerboseDataChunk
     */
    bool encodeVerboseDataChunk(uint8_t* buffer, size_t bufferSize,
                             const data::ChannelSample* samples, size_t count,
                             size_t& outputSize);
};

} // namespace network
} // namespace baja