#pragma once

#include <Arduino.h>
#include <QNEthernet.h>
#include <AsyncUDP_Teensy41.hpp>
#include <vector>
#include <string>
#include "util/ring_buffer.hpp"
#include "util/sample_data.hpp"
#include "adc/adc_channel_config.hpp"
#include "config/config.hpp"
#include "config/defines.h"
#include "serialization/pb_serializer.hpp"

namespace baja {
namespace network {

using namespace qindesign::network;

/**
 * @brief Asynchronous UDP client for telemetry data
 * 
 * Handles sending encoded protobuf messages to a server via UDP
 */
class AsyncUDPClient {
public:
    /**
     * @brief Construct a new Async UDP Client
     * 
     * @param encodedBuffer Reference to the buffer containing encoded protobuf messages
     */
    AsyncUDPClient(buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer);
    
    /**
     * @brief Destroy the Async UDP Client
     */
    ~AsyncUDPClient();
    
    /**
     * @brief Initialize UDP client
     * 
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @return true if initialization successful
     */
    bool begin(const char* serverAddress, uint16_t port = 8888);
    
    /**
     * @brief Initialize network connection with proper error handling
     * 
     * @return true if network initialized successfully
     */
    bool initializeNetwork();
    
    /**
     * @brief Convert IP address to string for logging
     * 
     * @param ip IP address to convert
     * @return String representation of the IP address
     */
    static String ipToString(IPAddress ip);
    
    /**
     * @brief Process encoded messages from the buffer
     * 
     * This function should be called regularly from the thread.
     * It reads encoded messages from the buffer and sends them via UDP.
     * 
     * @param maxMessages Maximum number of messages to process in one call (0 = unlimited)
     * @param maxTimeUs Maximum processing time in microseconds before yielding (0 = unlimited)
     * @return Number of messages processed
     */
    size_t process(size_t maxMessages = 0, uint32_t maxTimeUs = 0);
    
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
     * @brief Get the number of oversized messages dropped
     * 
     * @return Oversized message count
     */
    uint32_t getOversizedMessages() const { return oversizedMessages_; }
    
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
    // Reference to the encoded message buffer
    buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer_;
    
    // UDP client
    AsyncUDP udp_;
    
    // Server configuration
    char serverAddress_[64];
    uint16_t port_;
    bool isConnected_;
    
    // Statistics
    uint32_t messagesSent_;
    uint32_t oversizedMessages_;
    uint32_t bytesTransferred_;
    uint32_t sendErrors_;
    uint32_t lastStatsTime_;
    
    // Maximum UDP payload size (typical Ethernet MTU minus headers)
    static const size_t MAX_UDP_PAYLOAD = 1472;
    
    /**
     * @brief Send a single encoded message
     * 
     * @param encodedMsg The encoded message to send
     * @return true if send was successful
     */
    bool sendMessage(const serialization::EncodedMessage& encodedMsg);
    
    /**
     * @brief Check if the network is connected
     * 
     * @return true if connected
     */
    bool checkConnection();
    
    /**
     * @brief Log statistics periodically
     */
    void logStats();
};

} // namespace network
} // namespace baja