#include "network/udp_thread.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace network {

// Initialize static class members
AsyncUDPClient* UDPThread::udpClient_ = nullptr;
int UDPThread::threadId_ = -1;
volatile bool UDPThread::running_ = false;

// Statistics
uint32_t UDPThread::messagesProcessed_ = 0;
uint32_t UDPThread::bytesSent_ = 0;
uint32_t UDPThread::lastStatsTime_ = 0;

// Benchmarking metrics
uint32_t UDPThread::messageProcessingTimes_[100] = {0};
uint8_t UDPThread::timeIndex_ = 0;
uint32_t UDPThread::maxProcessingTime_ = 0;
uint32_t UDPThread::minProcessingTime_ = UINT32_MAX;
uint32_t UDPThread::totalProcessingTime_ = 0;
uint32_t UDPThread::benchmarkStartTime_ = 0;

bool UDPThread::initialize(
    buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer,
    const char* serverAddress,
    uint16_t port) {
    
    util::Debug::info("UDP Thread: Initializing");
    
    // Create the UDP client
    udpClient_ = new AsyncUDPClient(encodedBuffer);
    
    if (!udpClient_) {
        util::Debug::error("UDP Thread: Failed to create UDP client");
        return false;
    }
    
    // Initialize network
    if (!udpClient_->initializeNetwork()) {
        util::Debug::error("UDP Thread: Network initialization failed");
        return false;
    }
    
    // Initialize UDP client with server address and port
    if (!udpClient_->begin(serverAddress, port)) {
        util::Debug::error("UDP Thread: UDP client initialization failed");
        return false;
    }
    
    // Reset benchmarking metrics
    benchmarkStartTime_ = millis();
    messagesProcessed_ = 0;
    bytesSent_ = 0;
    maxProcessingTime_ = 0;
    minProcessingTime_ = UINT32_MAX;
    totalProcessingTime_ = 0;
    timeIndex_ = 0;
    
    util::Debug::info("UDP Thread: Initialization successful to " + String(serverAddress) + ":" + String(port));
    return true;
}

int UDPThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("UDP Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Check if UDP client is initialized
    if (!udpClient_) {
        util::Debug::error("UDP Thread: UDP client not initialized");
        return -1;
    }
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 4096);  // 4KB stack should be sufficient
    
    if (threadId_ <= 0) {
        util::Debug::error("UDP Thread: Failed to create thread");
        running_ = false;
        return -2;
    }
    
    // Set thread priority
    threads.setTimeSlice(threadId_, 5);  // Medium priority
    
    util::Debug::info("UDP Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool UDPThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    threadId_ = -1;
    
    util::Debug::info("UDP Thread: Stopped");
    return true;
}

bool UDPThread::isRunning() {
    return running_ && threadId_ > 0;
}

AsyncUDPClient* UDPThread::getClient() {
    return udpClient_;
}

void UDPThread::getStats(uint32_t& messagesSent, uint32_t& oversizedMessages, 
                       uint32_t& bytesTransferred, uint32_t& sendErrors) {
    if (udpClient_) {
        messagesSent = udpClient_->getMessagesSent();
        oversizedMessages = udpClient_->getOversizedMessages();
        bytesTransferred = udpClient_->getBytesTransferred();
        sendErrors = udpClient_->getSendErrors();
    } else {
        messagesSent = 0;
        oversizedMessages = 0;
        bytesTransferred = 0;
        sendErrors = 0;
    }
}

void UDPThread::recordProcessingTime(uint32_t processingTime) {
    // Store time in circular buffer
    messageProcessingTimes_[timeIndex_] = processingTime;
    timeIndex_ = (timeIndex_ + 1) % 100;
    
    // Update stats
    totalProcessingTime_ += processingTime;
    if (processingTime > maxProcessingTime_) {
        maxProcessingTime_ = processingTime;
    }
    if (processingTime < minProcessingTime_) {
        minProcessingTime_ = processingTime;
    }
}

void UDPThread::printBenchmarks() {
    // Calculate elapsed time since benchmark start
    uint32_t elapsed = millis() - benchmarkStartTime_;
    if (elapsed == 0) elapsed = 1; // Avoid division by zero
    
    // Get current stats
    uint32_t messagesSent = 0, oversizedMessages = 0, bytesTransferred = 0, sendErrors = 0;
    getStats(messagesSent, oversizedMessages, bytesTransferred, sendErrors);
    
    // Calculate messages per second
    float msgsPerSec = messagesSent * 1000.0f / elapsed;
    
    // Calculate bytes per second
    float kbytesPerSec = bytesTransferred * 1000.0f / elapsed / 1024.0f;
    
    // Calculate average processing time
    float avgProcessingTime = messagesProcessed_ > 0 ? totalProcessingTime_ / (float)messagesProcessed_ : 0;
    
    // Print benchmark results
    util::Debug::info("--------- UDP THREAD BENCHMARKS ---------");
    util::Debug::info("Messages sent: " + String(messagesSent) + 
                    " (" + String(msgsPerSec, 2) + " msgs/sec)");
    util::Debug::info("Total data: " + String(bytesTransferred / 1024.0f, 2) + 
                    " KB (" + String(kbytesPerSec, 2) + " KB/sec)");
    
    if (messagesProcessed_ > 0) {
        util::Debug::info("Processing time: avg=" + String(avgProcessingTime, 2) + 
                        "µs, min=" + String(minProcessingTime_) + 
                        "µs, max=" + String(maxProcessingTime_) + "µs");
    }
    
    if (oversizedMessages > 0) {
        util::Debug::warning("Oversized messages dropped: " + String(oversizedMessages));
    }
    
    if (sendErrors > 0) {
        util::Debug::warning("Send errors: " + String(sendErrors));
    }
    
    util::Debug::info("------------------------------------------");
}

void UDPThread::threadFunction(void* arg) {
    util::Debug::info("UDP Thread: Thread started");
    
    // Start benchmark timer
    benchmarkStartTime_ = millis();
    
    // Wait a moment for everything to stabilize
    delay(500);
    
    // Rate limiting
    const uint32_t MIN_SEND_PERIOD_MS = 10;  // Minimum time between sends (100Hz max)
    uint32_t lastSendTime = 0;
    
    uint32_t lastBenchmarkTime = 0;
    
    // Main thread loop
    while (running_) {
        uint32_t currentTime = millis();
        bool timeToSend = (currentTime - lastSendTime) >= MIN_SEND_PERIOD_MS;
        
        // Only process a message if enough time has elapsed
        if (timeToSend && udpClient_) {
            // Process start time
            uint32_t startMicros = micros();
            
            // Process exactly ONE message
            size_t processed = udpClient_->process(1, 0);
            
            if (processed > 0) {
                // Update benchmark metrics
                uint32_t processingTime = micros() - startMicros;
                recordProcessingTime(processingTime);
                messagesProcessed_ += processed;
                lastSendTime = currentTime;
                
                // Get latest stats for byte count
                uint32_t messagesSent, oversizedMessages, bytesTransferred, sendErrors;
                getStats(messagesSent, oversizedMessages, bytesTransferred, sendErrors);
                bytesSent_ = bytesTransferred;
            }
        }
        
        // Print benchmarks periodically (every 30 seconds)
        if (currentTime - lastBenchmarkTime >= 30000) {
            lastBenchmarkTime = currentTime;
            printBenchmarks();
        }
        
        // Always yield after each iteration to prevent hogging CPU
        threads.yield();
    }
    
    // Print final benchmarks before ending
    printBenchmarks();
    
    util::Debug::info("UDP Thread: Thread ended");
}

} // namespace network
} // namespace baja