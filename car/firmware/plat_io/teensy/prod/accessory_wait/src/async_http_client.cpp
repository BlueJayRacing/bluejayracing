#include "async_http_client.hpp"
#include <ArduinoJson.h>
#include "debug_util.hpp"

namespace baja {
namespace network {

// State machine states for HTTP client
enum HttpClientState {
    STATE_IDLE,
    STATE_CONNECTING,
    STATE_SENDING,
    STATE_WAITING,
    STATE_READING,
    STATE_PROCESSING,
    STATE_CLOSING
};

// Global state variables (static to persist between calls)
static HttpClientState currentState = STATE_IDLE;
static uint32_t stateStartTime = 0;
static size_t bytesSent = 0;
static String responseBuffer;
static int statusCode = -1;
static EthernetClient client;
static bool firstAttempt = true;

AsyncHTTPClient::AsyncHTTPClient(buffer::RingBuffer<data::ChannelSample, config::SAMPLE_RING_BUFFER_SIZE>& ringBuffer)
    : ringBuffer_(ringBuffer),
      requestPtr_(nullptr),
      port_(80),
      downsampleRatio_(1),
      sampleCounter_(0),
      lastReadPosition_(0),
      requestInProgress_(false),
      lastRequestTime_(0),
      lastResponseCode_(0),
      lastErrorReportTime_(0),
      lastReadyState_(readyStateUnsent),
      networkInitialized_(false),
      retryCount_(0),
      channelConfigs_(nullptr),
      successCount_(0),
      errorCount_(0) {
    
    // Initialize default server address
    strncpy(serverAddress_, "localhost", sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    
    // Initialize default endpoint
    strncpy(endpoint_, "/api/data", sizeof(endpoint_) - 1);
    endpoint_[sizeof(endpoint_) - 1] = '\0';
    
    // Create the request object (for compatibility, not used)
    requestPtr_ = new AsyncHTTPRequest();
    
    if (!requestPtr_) {
        util::Debug::error("Failed to create AsyncHTTPRequest object");
    }
    
    util::Debug::info("AsyncHTTPClient: Client initialized");
}

AsyncHTTPClient::~AsyncHTTPClient() {
    // Clean up the request object
    if (requestPtr_) {
        delete requestPtr_;
        requestPtr_ = nullptr;
    }
}

bool AsyncHTTPClient::begin(const char* serverAddress, uint16_t port, const char* endpoint) {
    // Store server configuration
    strncpy(serverAddress_, serverAddress, sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    
    strncpy(endpoint_, endpoint, sizeof(endpoint_) - 1);
    endpoint_[sizeof(endpoint_) - 1] = '\0';
    
    port_ = port;
    
    util::Debug::info("AsyncHTTPClient: Initializing for " + String(serverAddress_) + ":" + String(port_) + endpoint_);
    
    // Set up the QNEthernet connection if not already initialized
    if (!networkInitialized_) {
        if (!initializeNetwork()) {
            util::Debug::error("AsyncHTTPClient: Network initialization failed!");
            return false;
        }
    }
    
    // Reset state machine
    currentState = STATE_IDLE;
    stateStartTime = 0;
    requestInProgress_ = false;
    
    // First request will be sent after a short delay
    lastRequestTime_ = millis() - (HTTP_REQUEST_INTERVAL_MS - 1000);
    
    util::Debug::info("AsyncHTTPClient: Initialization complete");
    return true;
}

bool AsyncHTTPClient::initializeNetwork() {
    util::Debug::info("AsyncHTTPClient: Initializing QNEthernet...");
    
    // Check if Ethernet is already initialized
    if (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp && Ethernet.localIP() != IPAddress(0, 0, 0, 0)) {
        util::Debug::info("AsyncHTTPClient: Network already initialized with IP: " + ipToString(Ethernet.localIP()));
        networkInitialized_ = true;
        return true;
    }
    
    // First end any existing connections to ensure clean state
    Ethernet.end();
    delay(100);
    
    #if USING_DHCP
        util::Debug::info("AsyncHTTPClient: Using DHCP...");
        if (!Ethernet.begin()) {
            util::Debug::error("AsyncHTTPClient: DHCP configuration failed");
            return false;
        }
        
        // Wait for DHCP to complete
        if (!Ethernet.waitForLocalIP(5000)) {
            util::Debug::error("AsyncHTTPClient: DHCP timeout");
            return false;
        }
    #else
        util::Debug::info("AsyncHTTPClient: Using static IP...");
        
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
    util::Debug::info("AsyncHTTPClient: Waiting for network link...");
    while (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp && millis() - startTime < 5000) {
        delay(100);
    }
    
    if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
        util::Debug::error("AsyncHTTPClient: Ethernet link not established");
        return false;
    }
    
    // Get and log the IP address
    IPAddress localIP = Ethernet.localIP();
    util::Debug::info("AsyncHTTPClient: Connected with IP: " + ipToString(localIP));
    
    networkInitialized_ = true;
    return true;
}

String AsyncHTTPClient::ipToString(IPAddress ip) {
    return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

void AsyncHTTPClient::setDownsampleRatio(uint8_t ratio) {
    if (ratio > 0) {
        downsampleRatio_ = ratio;
        util::Debug::info("AsyncHTTPClient: Downsample ratio set to " + String(ratio));
    }
}

void AsyncHTTPClient::setChannelConfigs(baja::adc::ChannelConfig* channelConfigs) {
    channelConfigs_ = channelConfigs;
    util::Debug::info("AsyncHTTPClient: Channel configs set");
}

void AsyncHTTPClient::setHeader(const char* headerName, const char* headerValue) {
    // Add or update header
    for (auto& header : customHeaders_) {
        if (strcmp(header.name, headerName) == 0) {
            strncpy(header.value, headerValue, sizeof(header.value) - 1);
            header.value[sizeof(header.value) - 1] = '\0';
            return;
        }
    }
    
    // Add new header
    Header newHeader;
    strncpy(newHeader.name, headerName, sizeof(newHeader.name) - 1);
    newHeader.name[sizeof(newHeader.name) - 1] = '\0';
    
    strncpy(newHeader.value, headerValue, sizeof(newHeader.value) - 1);
    newHeader.value[sizeof(newHeader.value) - 1] = '\0';
    
    customHeaders_.push_back(newHeader);
    util::Debug::info("AsyncHTTPClient: Added header " + String(headerName) + ": " + String(headerValue));
}

size_t AsyncHTTPClient::process() {
    // First check if network is connected
    if (!checkConnection()) {
        if (millis() - lastErrorReportTime_ > ERROR_REPORT_INTERVAL_MS) {
            util::Debug::warning("AsyncHTTPClient: Network connection is down. Attempting to reconnect...");
            lastErrorReportTime_ = millis();
            
            if (!reconnect()) {
                return 0;
            }
        }
        return 0;
    }
    
    // Handle ongoing request state machine
    if (currentState != STATE_IDLE) {
        // Continue processing the state machine
        bool result = continueDirectRequest();
        return result ? 1 : 0;
    }
    
    // If we're here, we're not in an active request
    // Check if enough time has passed since last request (rate limiting)
    uint32_t currentTime = millis();
    if (currentTime - lastRequestTime_ < HTTP_REQUEST_INTERVAL_MS) {
        return 0;
    }
    
    // Get number of available samples in the ring buffer
    size_t availableSamples = ringBuffer_.available();
    if (availableSamples == 0) {
        return 0;
    }
    
    // Determine how many samples to process (limit to a reasonable batch size)
    size_t maxSamples = 10; // Smaller batch size for testing
    size_t samplesToProcess = min(availableSamples, maxSamples);
    
    util::Debug::info("AsyncHTTPClient: Processing " + String(samplesToProcess) + " of " + 
                     String(availableSamples) + " available samples");
    
    // Create a temporary buffer for the samples we'll process
    data::ChannelSample* samples = new data::ChannelSample[samplesToProcess];
    if (!samples) {
        util::Debug::error("AsyncHTTPClient: Failed to allocate sample buffer");
        return 0;
    }
    
    // Read samples from the ring buffer (using peek to not remove them yet)
    size_t sampleIndex = 0;
    for (size_t i = 0; i < samplesToProcess; i++) {
        data::ChannelSample sample;
        if (ringBuffer_.peek(sample, i + lastReadPosition_)) {
            if (sampleCounter_++ % downsampleRatio_ == 0) {
                samples[sampleIndex++] = sample;
            }
        }
    }
    
    // Build the JSON batch for the samples
    size_t processedCount = 0;
    if (sampleIndex > 0) {
        if (buildJsonBatch(samples, sampleIndex)) {
            // Start the direct request state machine
            if (startDirectRequest()) {
                // Mark the samples as processed
                lastReadPosition_ += samplesToProcess;
                processedCount = sampleIndex;
                
                // Update timing
                lastRequestTime_ = currentTime;
                requestInProgress_ = true;
                
                util::Debug::info("AsyncHTTPClient: Direct request started with " + String(sampleIndex) + " samples");
            } else {
                util::Debug::warning("AsyncHTTPClient: Failed to start direct request");
            }
        }
    }
    
    // Free the temporary buffer
    delete[] samples;
    
    return processedCount;
}

bool AsyncHTTPClient::buildJsonBatch(const data::ChannelSample* samples, size_t count) {
    // Create a DynamicJsonDocument
    // Size calculation: ~100 bytes overhead + ~60 bytes per sample
    const size_t jsonCapacity = 100 + (count * 60);
    
    DynamicJsonDocument doc(jsonCapacity);
    
    // Add metadata
    doc["device"] = "Teensy41";
    doc["timestamp"] = millis();
    doc["count"] = count;
    doc["ip"] = ipToString(Ethernet.localIP());
    
    // Create samples array
    JsonArray samplesArray = doc.createNestedArray("samples");
    
    // Add each sample
    for (size_t i = 0; i < count; i++) {
        JsonObject sample = samplesArray.createNestedObject();
        sample["ts"] = samples[i].timestamp;
        sample["ch"] = samples[i].channelIndex;
        sample["val"] = samples[i].rawValue;
        
        // Add channel name if available
        if (channelConfigs_ && samples[i].channelIndex < adc::ADC_CHANNEL_COUNT) {
            const std::string& name = channelConfigs_[samples[i].channelIndex].name;
            if (!name.empty()) {
                sample["name"] = name.c_str();
            }
        }
    }
    
    // Serialize to the message buffer
    size_t jsonLen = serializeJson(doc, messageBuffer_, data::MQTT_OPTIMAL_MESSAGE_SIZE - 1);
    
    // Ensure null termination
    messageBuffer_[jsonLen] = '\0';
    
    // Log the message size
    util::Debug::detail("AsyncHTTPClient: JSON payload size: " + String(jsonLen) + " bytes");
    
    // Check if the JSON was too large for the buffer
    if (jsonLen >= data::MQTT_OPTIMAL_MESSAGE_SIZE - 1) {
        util::Debug::warning("AsyncHTTPClient: JSON payload truncated! Consider using smaller batch size.");
    }
    
    return true;
}

// Start a new direct HTTP request using the state machine
bool AsyncHTTPClient::startDirectRequest() {
    // Parse the server IP address
    IPAddress serverIP;
    int a, b, c, d;
    if (sscanf(serverAddress_, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
        serverIP = IPAddress(a, b, c, d);
    } else {
        int result = Ethernet.hostByName(serverAddress_, serverIP);
        if (result <= 0) {
            util::Debug::error("AsyncHTTPClient: Failed to resolve hostname: " + String(serverAddress_));
            return false;
        }
    }
    
    // Reset state
    client.stop(); // Make sure we start fresh
    responseBuffer = "";
    statusCode = -1;
    bytesSent = 0;
    
    // Start connecting
    util::Debug::info("AsyncHTTPClient: Connecting to " + String(serverAddress_) + ":" + String(port_));
    currentState = STATE_CONNECTING;
    stateStartTime = millis();
    
    // Try to connect (non-blocking)
    if (!client.connect(serverIP, port_)) {
        // If connect returns false, it might be because it's in progress or failed immediately
        if (client.connected()) {
            util::Debug::info("AsyncHTTPClient: Connection in progress");
        } else {
            util::Debug::error("AsyncHTTPClient: Connection failed immediately");
            currentState = STATE_IDLE;
            errorCount_++;
            return false;
        }
    } else {
        util::Debug::info("AsyncHTTPClient: Connected immediately");
    }
    
    requestInProgress_ = true;
    return true;
}

// Continue processing an ongoing HTTP request
bool AsyncHTTPClient::continueDirectRequest() {
    // Check for timeout
    if (millis() - stateStartTime > 10000) { // 10 second timeout
        util::Debug::warning("AsyncHTTPClient: Request timed out in state " + String(currentState));
        client.stop();
        currentState = STATE_IDLE;
        requestInProgress_ = false;
        errorCount_++;
        return false;
    }
    
    // Process based on current state
    switch (currentState) {
        case STATE_CONNECTING:
            if (client.connected()) {
                util::Debug::info("AsyncHTTPClient: Connection established, sending request");
                currentState = STATE_SENDING;
                stateStartTime = millis();
                bytesSent = 0;
            } else {
                // Still trying to connect
                util::Debug::detail("AsyncHTTPClient: Waiting for connection...");
            }
            break;
            
        case STATE_SENDING: {
            if (bytesSent == 0) {
                // Build and send HTTP headers
                util::Debug::detail("AsyncHTTPClient: Sending HTTP headers");
                
                // Request line
                client.print("POST ");
                client.print(endpoint_);
                client.println(" HTTP/1.1");
                
                // Host header
                client.print("Host: ");
                client.println(serverAddress_);
                
                // Content-Type
                client.println("Content-Type: application/json");
                
                // Content-Length
                client.print("Content-Length: ");
                client.println(strlen(messageBuffer_));
                
                // Custom headers
                for (const auto& header : customHeaders_) {
                    client.print(header.name);
                    client.print(": ");
                    client.println(header.value);
                }
                
                // End of headers
                client.println("Connection: close");
                client.println();
                
                // Log what we're sending (first time only)
                if (firstAttempt) {
                    util::Debug::info("AsyncHTTPClient: Sending HTTP request:");
                    util::Debug::info("POST " + String(endpoint_) + " HTTP/1.1");
                    util::Debug::info("Host: " + String(serverAddress_));
                    util::Debug::info("Content-Type: application/json");
                    util::Debug::info("Content-Length: " + String(strlen(messageBuffer_)));
                    firstAttempt = false;
                } else {
                    util::Debug::detail("AsyncHTTPClient: Headers sent");
                }
                
                // Send payload
                util::Debug::detail("AsyncHTTPClient: Sending JSON payload");
                client.print(messageBuffer_);
                
                bytesSent = strlen(messageBuffer_);
                currentState = STATE_WAITING;
                stateStartTime = millis();
                
                util::Debug::info("AsyncHTTPClient: Request sent, waiting for response");
            }
            break;
        }
            
        case STATE_WAITING:
            if (client.available()) {
                util::Debug::info("AsyncHTTPClient: Response starting to arrive");
                currentState = STATE_READING;
                stateStartTime = millis();
                responseBuffer = "";
            } else if (!client.connected()) {
                // Connection closed without data
                util::Debug::warning("AsyncHTTPClient: Connection closed before response");
                currentState = STATE_CLOSING;
                stateStartTime = millis();
            }
            break;
            
        case STATE_READING: {
            if (client.available()) {
                // Read the status line first if we haven't yet
                if (statusCode == -1) {
                    String statusLine = client.readStringUntil('\n');
                    statusLine.trim();
                    
                    // Extract status code
                    if (statusLine.startsWith("HTTP/1.")) {
                        statusCode = statusLine.substring(9, 12).toInt();
                        util::Debug::info("AsyncHTTPClient: Status code: " + String(statusCode));
                    }
                }
                
                // Read more data in chunks
                int bytesToRead = min(client.available(), 100);
                char buffer[101];
                int bytesRead = 0;
                
                for (int i = 0; i < bytesToRead; i++) {
                    buffer[bytesRead++] = client.read();
                }
                
                buffer[bytesRead] = '\0';
                responseBuffer += buffer;
                
                // Keep the response buffer from growing too large
                if (responseBuffer.length() > 1024) {
                    responseBuffer = responseBuffer.substring(0, 1024) + "...";
                }
                
                // Reset timeout
                stateStartTime = millis();
                
                util::Debug::detail("AsyncHTTPClient: Read " + String(bytesRead) + " response bytes");
            } else if (!client.connected()) {
                // No more data and connection closed
                util::Debug::info("AsyncHTTPClient: Response complete, processing");
                currentState = STATE_PROCESSING;
                stateStartTime = millis();
            }
            break;
        }
            
        case STATE_PROCESSING: {
            // Process the response
            if (statusCode >= 200 && statusCode < 300) {
                util::Debug::info("AsyncHTTPClient: Request successful (status " + String(statusCode) + ")");
                
                // Extract response body (simple approach)
                int bodyStart = responseBuffer.indexOf("\r\n\r\n");
                String responseBody = bodyStart > 0 ? responseBuffer.substring(bodyStart + 4) : responseBuffer;
                
                // Log response body (truncated if long)
                if (responseBody.length() > 100) {
                    util::Debug::info("AsyncHTTPClient: Response: " + responseBody.substring(0, 100) + "...");
                } else if (responseBody.length() > 0) {
                    util::Debug::info("AsyncHTTPClient: Response: " + responseBody);
                } else {
                    util::Debug::info("AsyncHTTPClient: Empty response body");
                }
                
                lastResponseCode_ = statusCode;
                successCount_++;
            } else {
                util::Debug::warning("AsyncHTTPClient: Request failed with status " + String(statusCode));
                lastResponseCode_ = statusCode;
                errorCount_++;
            }
            
            // Move to closing state
            currentState = STATE_CLOSING;
            stateStartTime = millis();
            break;
        }
            
        case STATE_CLOSING:
            client.stop();
            util::Debug::info("AsyncHTTPClient: Connection closed");
            
            // Reset state
            currentState = STATE_IDLE;
            requestInProgress_ = false;
            
            util::Debug::info("AsyncHTTPClient: Request complete (Success: " + 
                            String(successCount_) + ", Errors: " + String(errorCount_) + ")");
            break;
            
        default:
            util::Debug::error("AsyncHTTPClient: Invalid state " + String(currentState));
            currentState = STATE_IDLE;
            requestInProgress_ = false;
            return false;
    }
    
    return true;
}

// Legacy method (not used)
bool AsyncHTTPClient::sendRequest() {
    return false;
}

// Legacy method (not used)
bool AsyncHTTPClient::sendDirectRequest() {
    return false;
}

// Legacy callback (not used)
void AsyncHTTPClient::requestCallback(void* optParm, AsyncHTTPRequest* request, int readyState) {
    // Not used in the new implementation
}

bool AsyncHTTPClient::isConnected() const {
    return networkInitialized_ && Ethernet.linkStatus() == LinkStatus_kLinkStatusUp;
}

bool AsyncHTTPClient::checkConnection() {
    if (!isConnected()) {
        return false;
    }
    
    // Just check if link status is up and we have an IP
    IPAddress ip = Ethernet.localIP();
    bool validIP = !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
    
    return validIP && (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp);
}

bool AsyncHTTPClient::reconnect() {
    // Limit retries
    if (retryCount_ > HTTP_MAX_RETRIES) {
        util::Debug::error("AsyncHTTPClient: Max retry count exceeded, backing off");
        
        // Reset after longer timeout
        if (millis() - lastRequestTime_ > 60000) {
            retryCount_ = 0;
        }
        return false;
    }
    
    util::Debug::info("AsyncHTTPClient: Attempting to reconnect (attempt #" + String(retryCount_ + 1) + ")");
    
    // Reset network connection
    networkInitialized_ = false;
    
    // Reinitialize network
    bool success = initializeNetwork();
    
    if (success) {
        util::Debug::info("AsyncHTTPClient: Reconnection successful");
        lastRequestTime_ = millis() - (HTTP_REQUEST_INTERVAL_MS - 2000); // Allow a request soon
    } else {
        util::Debug::error("AsyncHTTPClient: Reconnection failed");
        retryCount_++;
    }
    
    return success;
}

std::string AsyncHTTPClient::getChannelName(uint8_t channelIndex) const {
    if (channelConfigs_ && channelIndex < adc::ADC_CHANNEL_COUNT) {
        return channelConfigs_[channelIndex].name;
    }
    return "";
}

} // namespace network
} // namespace baja