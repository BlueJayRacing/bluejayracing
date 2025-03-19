#include "network/http_thread.hpp"
#include "util/debug_util.hpp"


namespace baja {
namespace network {

// Initialize static class members
AsyncHTTPClient* HTTPThread::httpClient_ = nullptr;
int HTTPThread::threadId_ = -1;
volatile bool HTTPThread::running_ = false;
uint32_t HTTPThread::requestsCount_ = 0;
uint32_t HTTPThread::successCount_ = 0;
uint32_t HTTPThread::errorCount_ = 0;

bool HTTPThread::initialize(
    buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer,
    const char* serverAddress,
    uint16_t port,
    const char* endpoint) {
    
    util::Debug::info("HTTP Thread: Initializing");
    
    // Create the HTTP client with encoded buffer
    httpClient_ = new AsyncHTTPClient(encodedBuffer);
    
    if (!httpClient_) {
        util::Debug::error("HTTP Thread: Failed to create HTTP client");
        return false;
    }
    
    // Initialize network
    if (!httpClient_->initializeNetwork()) {
        util::Debug::error("HTTP Thread: Network initialization failed");
        return false;
    }
    
    // Initialize HTTP client with persistent connections
    if (!httpClient_->begin(serverAddress, port, endpoint, true)) {
        util::Debug::error("HTTP Thread: HTTP client initialization failed");
        return false;
    }
    
    util::Debug::info("HTTP Thread: Initialization successful with persistent connections");
    return true;
}

int HTTPThread::start() {
    // Check if already running
    if (running_ || threadId_ > 0) {
        util::Debug::warning("HTTP Thread: Already running with ID " + String(threadId_));
        return threadId_;
    }
    
    // Check if HTTP client is initialized
    if (!httpClient_) {
        util::Debug::error("HTTP Thread: HTTP client not initialized");
        return -1;
    }
    
    // Reset counters
    requestsCount_ = 0;
    successCount_ = 0;
    errorCount_ = 0;
    
    // Create the thread
    running_ = true;
    threadId_ = threads.addThread(threadFunction, 0, 8192);  // Larger stack for HTTP
    
    if (threadId_ <= 0) {
        util::Debug::error("HTTP Thread: Failed to create thread");
        running_ = false;
        return -2;
    }
    
    // Set thread priority
    threads.setTimeSlice(threadId_, 3);  // Medium priority
    
    util::Debug::info("HTTP Thread: Started with ID " + String(threadId_));
    return threadId_;
}

bool HTTPThread::stop() {
    if (!running_ || threadId_ <= 0) {
        return false;
    }
    
    // Signal thread to stop
    running_ = false;
    
    // Wait for thread to finish
    threads.wait(threadId_, 1000);  // Wait up to 1 second
    threadId_ = -1;
    
    util::Debug::info("HTTP Thread: Stopped");
    return true;
}

bool HTTPThread::isRunning() {
    return running_ && threadId_ > 0;
}

AsyncHTTPClient* HTTPThread::getClient() {
    return httpClient_;
}

void HTTPThread::setChannelConfigs(adc::ChannelConfig* channelConfigs) {
    if (!httpClient_) {
        util::Debug::error("HTTP Thread: HTTP client not initialized");
        return;
    }
    
    httpClient_->setChannelConfigs(channelConfigs);
}

void HTTPThread::setDownsampleRatio(uint8_t ratio) {
    if (!httpClient_) {
        util::Debug::error("HTTP Thread: HTTP client not initialized");
        return;
    }
    
    httpClient_->setDownsampleRatio(ratio);
}

void HTTPThread::threadFunction(void* arg) {
    util::Debug::info("HTTP Thread: Thread started");
    
    // Wait for things to stabilize before sending first request
    delay(2000);
    
    uint32_t lastProcessTime = 0;
    uint32_t lastStatsTime = 0;
    uint32_t requestsAttempted = 0;
    uint32_t requestsSucceeded = 0;
    
    // Main thread loop
    while (running_) {
        uint32_t currentTime = millis();
        
        // Process data as frequently as possible
        // Let the client's rate limiting handle delays
        if (currentTime - lastProcessTime >= 5) {  // Try every 5ms for more responsive processing
            lastProcessTime = currentTime;
            
            // Try to process encoded messages from the buffer
            if (httpClient_) {
                size_t processed = httpClient_->process();
                
                if (processed > 0) {
                    requestsAttempted++;
                    requestsSucceeded++;
                }
            }
        }
        
        // Print stats every 30 seconds
        if (currentTime - lastStatsTime >= 30000) {
            lastStatsTime = currentTime;
            
            util::Debug::info("HTTP Thread: Stats - Attempted: " + 
                String(requestsAttempted) + ", Succeeded: " + 
                String(requestsSucceeded) + ", Rate: " + 
                String((float)requestsSucceeded * 1000 / 30000, 1) + " msg/sec");
                
            // Reset counters for next period
            requestsAttempted = 0;
            requestsSucceeded = 0;
                
            // Check network status
            if (httpClient_) {
                bool connected = httpClient_->isConnected();
                util::Debug::info("HTTP Thread: Network status: " + 
                    String(connected ? "Connected" : "Disconnected"));
                
                // If not connected, try to reconnect
                if (!connected) {
                    util::Debug::info("HTTP Thread: Attempting to reconnect...");
                    if (httpClient_->reconnect()) {
                        util::Debug::info("HTTP Thread: Reconnection successful");
                    } else {
                        util::Debug::error("HTTP Thread: Reconnection failed");
                        errorCount_++;
                    }
                }
            }
        }
        
        // Small yield to allow other threads to run
        threads.yield();
        
        // Add a small delay to prevent tight loop
        delay(5);
    }
    
    util::Debug::info("HTTP Thread: Thread ended");
}

} // namespace network
} // namespace baja
    