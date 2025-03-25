#pragma once

#include <TeensyThreads.h>
#include "network/async_udp_client.hpp"
#include "serialization/pb_serializer.hpp"
#include <QNEthernet.h>
#include <AsyncUDP_Teensy41.hpp>

namespace baja {
namespace network {

/**
 * @brief UDP client thread module
 * 
 * Handles initialization, running, and monitoring of the UDP client thread.
 */
class UDPThread {
public:
    /**
     * @brief Initialize the UDP thread module
     * 
     * @param encodedBuffer Reference to the buffer containing encoded protobuf messages
     * @param serverAddress Server address (hostname or IP address)
     * @param port Server port
     * @return true if initialization was successful
     */
    static bool initialize(
        buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer,
        const char* serverAddress,
        uint16_t port = 8888);
    
    /**
     * @brief Start the UDP thread
     * 
     * @return Thread ID if successful, negative value if failed
     */
    static int start();
    
    /**
     * @brief Stop the UDP thread
     * 
     * @return true if successful
     */
    static bool stop();
    
    /**
     * @brief Check if the UDP thread is running
     * 
     * @return true if running
     */
    static bool isRunning();
    
    /**
     * @brief Get the UDP client instance
     * 
     * @return Pointer to the UDP client
     */
    static AsyncUDPClient* getClient();
    
    /**
     * @brief Get statistics about UDP client operation
     * 
     * @param messagesSent Output parameter for number of messages sent
     * @param oversizedMessages Output parameter for number of oversized messages dropped
     * @param bytesTransferred Output parameter for number of bytes transferred
     * @param sendErrors Output parameter for number of send errors
     */
    static void getStats(uint32_t& messagesSent, uint32_t& oversizedMessages, 
                        uint32_t& bytesTransferred, uint32_t& sendErrors);

private:
    static AsyncUDPClient* udpClient_;
    static int threadId_;
    static volatile bool running_;
    
    // Statistics
    static uint32_t messagesProcessed_;
    static uint32_t bytesSent_;
    static uint32_t lastStatsTime_;
    
    // Benchmarking metrics
    static uint32_t messageProcessingTimes_[100];
    static uint8_t timeIndex_;
    static uint32_t maxProcessingTime_;
    static uint32_t minProcessingTime_;
    static uint32_t totalProcessingTime_;
    static uint32_t benchmarkStartTime_;
    
    /**
     * @brief UDP client thread function
     * 
     * @param arg Thread argument (not used)
     */
    static void threadFunction(void* arg);
    
    /**
     * @brief Record processing time for benchmarking
     * 
     * @param processingTime Processing time in microseconds
     */
    static void recordProcessingTime(uint32_t processingTime);
    
    /**
     * @brief Print benchmark results
     */
    static void printBenchmarks();
};

} // namespace network
} // namespace baja