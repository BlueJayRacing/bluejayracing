#include "network/async_http_client.hpp"
#include <ArduinoJson.h>
#include "util/debug_util.hpp"

#include <AsyncHTTPRequest_Teensy41.h>

namespace baja {
namespace network {

// For prettier debug logging
static const char* stateNames[] = {
    "IDLE",
    "CONNECTING",
    "SENDING",
    "WAITING",
    "READING",
    "PROCESSING",
    "CLOSING",
    "KEEPALIVE"
};

// Maximum idle time for a persistent connection (30 seconds)
const uint32_t MAX_IDLE_CONNECTION_TIME = 30000;
// Maximum read time for response (5 seconds)
const uint32_t MAX_READ_TIME = 5000;
// Timeout for state transitions (10 seconds default, shorter for reading)
const uint32_t DEFAULT_STATE_TIMEOUT = 10000;

AsyncHTTPClient::AsyncHTTPClient(buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer)
    : encodedBuffer_(encodedBuffer),
      requestPtr_(nullptr),
      serverAddress_(),
      endpoint_(),
      port_(80),
      customHeaders_(),
      usePersistentConnection_(true),
      connectionEstablished_(false),
      idleConnectionTime_(0),
      connectionAttempts_(0),
      downsampleRatio_(1),
      sampleCounter_(0),
      currentState_(STATE_IDLE),
      stateStartTime_(0),
      lastReadPosition_(0),
      requestInProgress_(false),
      lastRequestTime_(0),
      lastResponseCode_(0),
      lastErrorReportTime_(0),
      lastReadyState_(readyStateUnsent),
      networkInitialized_(false),
      retryCount_(0),
      bytesSent_(0),
      responseBuffer_(""),
      statusCode_(-1),
      successCount_(0),
      errorCount_(0),
      timeoutCount_(0),
      requestCount_(0),
      channelConfigs_(nullptr),
      client_(),
      messageBuffer_(nullptr),
      messageBufferSize_(0) {
    
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
    
    // Allocate a reasonably sized initial message buffer (4KB)
    messageBufferSize_ = 4096;
    messageBuffer_ = new char[messageBufferSize_];
    if (!messageBuffer_) {
        util::Debug::error("Failed to allocate message buffer");
        messageBufferSize_ = 0;
    }
    
    util::Debug::info("AsyncHTTPClient: Client initialized with " + String(messageBufferSize_) + " byte buffer");
}

AsyncHTTPClient::~AsyncHTTPClient() {
    // Close any active connection
    closeConnection(true);
    
    // Clean up the request object
    if (requestPtr_) {
        delete requestPtr_;
        requestPtr_ = nullptr;
    }
    
    // Clean up message buffer
    if (messageBuffer_) {
        delete[] messageBuffer_;
        messageBuffer_ = nullptr;
        messageBufferSize_ = 0;
    }
}

bool AsyncHTTPClient::begin(const char* serverAddress, uint16_t port, const char* endpoint, 
                           bool usePersistentConnection) {
    // Store server configuration
    strncpy(serverAddress_, serverAddress, sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    
    strncpy(endpoint_, endpoint, sizeof(endpoint_) - 1);
    endpoint_[sizeof(endpoint_) - 1] = '\0';
    
    port_ = port;
    usePersistentConnection_ = usePersistentConnection;
    
    // Close any existing connection before changing settings
    closeConnection(true);
    connectionEstablished_ = false;
    
    util::Debug::info("AsyncHTTPClient: Initializing for " + String(serverAddress_) + ":" + 
                    String(port_) + endpoint_ + 
                    (usePersistentConnection_ ? " (persistent)" : " (non-persistent)"));
    
    // Set up the QNEthernet connection if not already initialized
    if (!networkInitialized_) {
        if (!initializeNetwork()) {
            util::Debug::error("AsyncHTTPClient: Network initialization failed!");
            return false;
        }
    }
    
    // Reset state machine
    currentState_ = STATE_IDLE;
    stateStartTime_ = 0;
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
    
    // Handle persistent connection management
    if (usePersistentConnection_) {
        managePersistentConnection();
    }
    
    // Handle ongoing request state machine
    if (currentState_ != STATE_IDLE && currentState_ != STATE_KEEPALIVE) {
        // Continue processing the state machine
        bool result = continueDirectRequest();
        return result ? 1 : 0;
    }
    
    // We're either in IDLE or KEEPALIVE state, so we can start a new request
    
    // Check if enough time has passed since last request (rate limiting)
    uint32_t currentTime = millis();
    if (currentTime - lastRequestTime_ < HTTP_REQUEST_INTERVAL_MS) {
        return 0;
    }
    
    // Get number of available encoded messages
    size_t availableMessages = encodedBuffer_.available();
    
    // Log buffer status periodically
    static uint32_t lastBufferCheckTime = 0;
    if (currentTime - lastBufferCheckTime > 5000) {  // Every 5 seconds
        util::Debug::info("AsyncHTTPClient: Encoded buffer has " + 
                       String(availableMessages) + "/" + 
                       String(encodedBuffer_.capacity()) + 
                       " messages (" + 
                       String(100.0f * availableMessages / encodedBuffer_.capacity(), 1) + "% full)");
        lastBufferCheckTime = currentTime;
    }
    
    if (availableMessages == 0) {
        return 0;
    }
    
    // Read an encoded message from the buffer
    serialization::EncodedMessage encodedMsg;
    if (!encodedBuffer_.read(encodedMsg)) {
        util::Debug::warning("AsyncHTTPClient: Failed to read encoded message from buffer");
        return 0;
    }
    
    // Debug: validate the message we just read
    if (encodedMsg.size == 0) {
        util::Debug::error("AsyncHTTPClient: Read empty encoded message from buffer");
        return 0;
    }
    
    util::Debug::info("AsyncHTTPClient: Processing encoded message of " + 
                     String(encodedMsg.size) + " bytes, timestamp=" + 
                     String(encodedMsg.timestamp));
    
    // Build the JSON wrapper for the encoded message
    if (buildJsonWrapper(encodedMsg)) {
        // Start the direct request state machine
        if (startDirectRequest()) {
            // Update timing
            lastRequestTime_ = currentTime;
            requestInProgress_ = true;
            
            util::Debug::info("AsyncHTTPClient: Direct request started with encoded message");
            return 1; // We processed one message
        } else {
            util::Debug::warning("AsyncHTTPClient: Failed to start direct request");
        }
    } else {
        util::Debug::error("AsyncHTTPClient: Failed to build JSON wrapper for encoded message");
    }
    
    return 0;
}

bool AsyncHTTPClient::buildJsonWrapper(const serialization::EncodedMessage& encodedMsg) {
    // Check if we have a valid encoded message
    if (encodedMsg.size == 0) {
        util::Debug::error("AsyncHTTPClient: Empty encoded message (size = 0)");
        return false;
    }
    
    // Log encoded message details
    util::Debug::info("AsyncHTTPClient: Building JSON wrapper for message of " + 
                   String(encodedMsg.size) + " bytes");
    
    // Calculate required buffer size for JSON
    // Each byte becomes 2 hex chars + metadata + some overhead
    size_t requiredSize = encodedMsg.size * 2 + 256;
    
    // Resize message buffer if needed
    if (requiredSize > messageBufferSize_) {
        util::Debug::info("AsyncHTTPClient: Resizing message buffer from " + 
                        String(messageBufferSize_) + " to " + String(requiredSize));
        char* newBuffer = new char[requiredSize];
        if (!newBuffer) {
            util::Debug::error("AsyncHTTPClient: Failed to allocate larger message buffer");
            return false;
        }
        
        delete[] messageBuffer_;
        messageBuffer_ = newBuffer;
        messageBufferSize_ = requiredSize;
    }
    
    // Create a DynamicJsonDocument sized for our data
    const size_t jsonCapacity = encodedMsg.size * 2 + 200;  // Double size for hex + overhead
    
    DynamicJsonDocument doc(jsonCapacity);
    
    // Add metadata
    doc["device"] = "Teensy41";
    doc["timestamp"] = millis();
    doc["type"] = "protobuf";
    doc["ip"] = ipToString(Ethernet.localIP());
    doc["size"] = encodedMsg.size;
    
    // Create hex string for binary data
    char* hexData = new char[encodedMsg.size * 2 + 1];
    if (!hexData) {
        util::Debug::error("AsyncHTTPClient: Failed to allocate memory for hex data");
        return false;
    }
    
    // Convert to hex string
    for (size_t i = 0; i < encodedMsg.size; i++) {
        sprintf(&hexData[i * 2], "%02X", encodedMsg.buffer[i]);
    }
    hexData[encodedMsg.size * 2] = '\0';
    
    // Add the encoded data to the JSON document
    doc["data"] = hexData;
    
    // Serialize to the message buffer with our dynamic size
    size_t jsonLen = serializeJson(doc, messageBuffer_, messageBufferSize_);
    
    // Clean up
    delete[] hexData;
    
    // Ensure null termination
    if (jsonLen < messageBufferSize_) {
        messageBuffer_[jsonLen] = '\0';
        
        // Log the JSON content for debugging
        util::Debug::info("AsyncHTTPClient: JSON wrapper size: " + String(jsonLen) + " bytes");
        
        // Print first part of the JSON for debugging
        String jsonPreview = String(messageBuffer_).substring(0, 100);
        if (jsonLen > 100) jsonPreview += "...";
        util::Debug::info("AsyncHTTPClient: JSON preview: " + jsonPreview);
        
        return true;
    } else {
        util::Debug::error("AsyncHTTPClient: JSON serialization failed - buffer too small");
        return false;
    }
}

std::string AsyncHTTPClient::getChannelName(uint8_t channelIndex) const {
    if (channelConfigs_ && channelIndex < adc::ADC_CHANNEL_COUNT) {
        return channelConfigs_[channelIndex].name;
    }
    return "";
}

// Start a new direct HTTP request using the state machine
bool AsyncHTTPClient::startDirectRequest() {
    // Make sure we're in the right state first
    if (currentState_ != STATE_IDLE && currentState_ != STATE_KEEPALIVE) {
        util::Debug::warning("AsyncHTTPClient: Cannot start new request in state " + getStateName());
        return false;
    }
    
    // Check for and handle existing connection
    if (currentState_ == STATE_KEEPALIVE) {
        // We're already connected, just reset some state variables
        bytesSent_ = 0;
        responseBuffer_ = "";
        statusCode_ = -1;
        
        // Move directly to sending state
        util::Debug::info("AsyncHTTPClient: Reusing existing connection, moving to SENDING state");
        currentState_ = STATE_SENDING;
        stateStartTime_ = millis();
        requestCount_++;
        return true;
    }
    
    // Not connected yet, establish a new connection
    if (!ensureConnection()) {
        util::Debug::error("AsyncHTTPClient: Failed to ensure connection");
        return false;
    }
    
    requestCount_++;
    return true;
}

// Legacy method (not used)
bool AsyncHTTPClient::sendRequest() {
    return false;
}

// Continue processing an ongoing HTTP request
bool AsyncHTTPClient::continueDirectRequest() {
    // Check for timeout based on state
    uint32_t stateTimeout = DEFAULT_STATE_TIMEOUT;
    
    // Use shorter timeout for reading state
    if (currentState_ == STATE_READING) {
        stateTimeout = MAX_READ_TIME;
    }
    
    if (millis() - stateStartTime_ > stateTimeout) {
        logTimeoutError();
        timeoutCount_++;
        closeConnection();
        resetState(true);
        return false;
    }
    
    // Process based on current state
    switch (currentState_) {
        case STATE_CONNECTING:
            if (client_.connected()) {
                util::Debug::info("AsyncHTTPClient: Connection established, sending request");
                currentState_ = STATE_SENDING;
                stateStartTime_ = millis();
                bytesSent_ = 0;
                connectionEstablished_ = true;
                connectionAttempts_ = 0;
            } else if (client_.status() < 0) {
                // Connection failed
                util::Debug::error("AsyncHTTPClient: Connection failed with status " + String(client_.status()));
                resetState(true);
                errorCount_++;
                return false;
            } else {
                // Still trying to connect
                util::Debug::detail("AsyncHTTPClient: Waiting for connection...");
            }
            break;
            
        case STATE_SENDING: {
            if (bytesSent_ == 0) {
                // Build and send HTTP headers
                util::Debug::detail("AsyncHTTPClient: Sending HTTP headers");
                
                // Request line
                client_.print("POST ");
                client_.print(endpoint_);
                client_.println(" HTTP/1.1");
                
                // Host header
                client_.print("Host: ");
                client_.println(serverAddress_);
                
                // Content-Type
                client_.println("Content-Type: application/json");
                
                // Content-Length
                client_.print("Content-Length: ");
                client_.println(strlen(messageBuffer_));
                
                // Connection type based on persistent setting
                client_.print("Connection: ");
                client_.println(usePersistentConnection_ ? "keep-alive" : "close");
                
                // Custom headers
                for (const auto& header : customHeaders_) {
                    client_.print(header.name);
                    client_.print(": ");
                    client_.println(header.value);
                }
                
                // End of headers
                client_.println();
                
                static bool firstAttempt = true;
                
                // Log what we're sending (first time only)
                if (firstAttempt) {
                    util::Debug::info("AsyncHTTPClient: Sending HTTP request:");
                    util::Debug::info("POST " + String(endpoint_) + " HTTP/1.1");
                    util::Debug::info("Host: " + String(serverAddress_));
                    util::Debug::info("Content-Type: application/json");
                    util::Debug::info("Content-Length: " + String(strlen(messageBuffer_)));
                    util::Debug::info("Connection: " + String(usePersistentConnection_ ? "keep-alive" : "close"));
                    firstAttempt = false;
                } else {
                    util::Debug::detail("AsyncHTTPClient: Headers sent");
                }
                
                // Send payload
                util::Debug::detail("AsyncHTTPClient: Sending JSON payload (" + 
                                 String(strlen(messageBuffer_)) + " bytes)");
                
                // Send in chunks if needed for large payloads
                const size_t MAX_CHUNK = 1024;
                size_t remaining = strlen(messageBuffer_);
                size_t offset = 0;
                
                while (remaining > 0) {
                    size_t chunkSize = min(remaining, MAX_CHUNK);
                    size_t sent = client_.write(messageBuffer_ + offset, chunkSize);
                    if (sent > 0) {
                        offset += sent;
                        remaining -= sent;
                    } else {
                        if (!client_.connected()) {
                            util::Debug::warning("AsyncHTTPClient: Connection lost during send");
                            resetState(true);
                            errorCount_++;
                            return false;
                        }
                        
                        util::Debug::warning("AsyncHTTPClient: Failed to send chunk, retry next time");
                        break;
                    }
                }
                
                bytesSent_ = strlen(messageBuffer_) - remaining;
                
                if (remaining == 0) {
                    currentState_ = STATE_WAITING;
                    util::Debug::info("AsyncHTTPClient: Request fully sent (" + 
                                   String(bytesSent_) + " bytes), waiting for response");
                } else {
                    util::Debug::warning("AsyncHTTPClient: Partial send (" + 
                                      String(bytesSent_) + " of " + 
                                      String(strlen(messageBuffer_)) + " bytes)");
                }
                
                stateStartTime_ = millis();
            }
            break;
        }
            
        case STATE_WAITING:
            if (client_.available()) {
                util::Debug::info("AsyncHTTPClient: Response starting to arrive");
                currentState_ = STATE_READING;
                stateStartTime_ = millis();
                responseBuffer_ = "";
            } else if (!client_.connected()) {
                // Connection closed without data
                util::Debug::warning("AsyncHTTPClient: Connection closed before response");
                resetState(true);
                errorCount_++;
                return false;
            }
            break;
            
        case STATE_READING: {
            if (client_.available()) {
                // Read the status line first if we haven't yet
                if (statusCode_ == -1) {
                    String statusLine = client_.readStringUntil('\n');
                    statusLine.trim();
                    
                    // Extract status code
                    if (statusLine.startsWith("HTTP/1.")) {
                        statusCode_ = statusLine.substring(9, 12).toInt();
                        util::Debug::info("AsyncHTTPClient: Status code: " + String(statusCode_));
                    }
                }
                
                // Read headers until we find the blank line that separates headers from body
                if (responseBuffer_.length() == 0) {
                    while (client_.available()) {
                        String line = client_.readStringUntil('\n');
                        line.trim();
                        
                        // Blank line means end of headers
                        if (line.length() == 0) {
                            break;
                        }
                    }
                }
                
                // At this point, we're in the body - just read all available data
                // For small responses, we can store them, but for larger responses
                // we'll just discard the data since we don't need it
                if (client_.available()) {
                    uint32_t bytesToRead = client_.available();
                    if (bytesToRead > 1024) {
                        // Just discard most of it, but keep the first bit for debugging
                        char discard[128];
                        
                        // Read first 128 bytes for logging
                        if (responseBuffer_.length() < 128) {
                            int bytesToKeep = min(bytesToRead, 128UL - responseBuffer_.length());
                            if (bytesToKeep > 0) {
                                client_.read((uint8_t*)discard, bytesToKeep);
                                discard[bytesToKeep] = '\0';
                                responseBuffer_ += discard;
                            }
                        }
                        
                        // Discard the rest
                        while (client_.available()) {
                            int bytesRead = client_.read((uint8_t*)discard, sizeof(discard));
                            if (bytesRead <= 0) break;
                        }
                        
                        util::Debug::detail("AsyncHTTPClient: Discarded large response body, kept first 128 bytes");
                    } else {
                        // For smaller responses, keep all of it (up to a reasonable limit)
                        const int MAX_RESPONSE = 1024;
                        
                        // Only read more if we haven't hit our limit
                        if (responseBuffer_.length() < MAX_RESPONSE) {
                            while (client_.available() && responseBuffer_.length() < MAX_RESPONSE) {
                                char c = client_.read();
                                responseBuffer_ += c;
                            }
                            
                            // Discard anything beyond our limit
                            while (client_.available()) {
                                client_.read();
                            }
                        } else {
                            // Discard excess data
                            while (client_.available()) {
                                client_.read();
                            }
                        }
                    }
                }
                
                // Reset timeout on data arrival
                stateStartTime_ = millis();
                
                // Check if we're done reading (connection closed or no more data)
                if (!client_.connected() && !client_.available()) {
                    util::Debug::info("AsyncHTTPClient: Response complete (connection closed)");
                    currentState_ = STATE_PROCESSING;
                    stateStartTime_ = millis();
                }
                
                // Special case: Check if server supports persistent connections
                // If it's keep-alive and we've got our status code, we can move to processing
                // once we have some data and there's a pause
                if (usePersistentConnection_ && statusCode_ > 0 && 
                    !client_.available() && client_.connected()) {
                    
                    // Wait a little bit for more data (50ms)
                    if (millis() - stateStartTime_ > 50) {
                        util::Debug::info("AsyncHTTPClient: Response appears complete (keep-alive pause)");
                        currentState_ = STATE_PROCESSING;
                        stateStartTime_ = millis();
                    }
                }
            } 
            else if (!client_.connected()) {
                // No more data and connection closed
                util::Debug::info("AsyncHTTPClient: Response complete, processing");
                currentState_ = STATE_PROCESSING;
                stateStartTime_ = millis();
            }
            break;
        }
            
        case STATE_PROCESSING: {
            // Process the response
            if (processResponse()) {
                // Move to KEEPALIVE or CLOSING state based on connection type
                if (usePersistentConnection_ && client_.connected()) {
                    // Keep connection open
                    currentState_ = STATE_KEEPALIVE;
                    idleConnectionTime_ = millis();
                    util::Debug::info("AsyncHTTPClient: Connection kept alive for reuse");
                } else {
                    // Close connection
                    currentState_ = STATE_CLOSING;
                }
                stateStartTime_ = millis();
            } else {
                // Processing failed, force close
                resetState(true);
                return false;
            }
            break;
        }
            
        case STATE_KEEPALIVE:
            // This is an idle state, nothing to do but monitor the connection
            // The managePersistentConnection method handles timeouts
            return true;
            
        case STATE_CLOSING:
            closeConnection();
            resetState();
            return true;
            
        case STATE_IDLE:
            // Nothing to do in idle state
            return true;
            
        default:
            util::Debug::error("AsyncHTTPClient: Invalid state " + String(currentState_));
            resetState(true);
            return false;
    }
    
    return true;
}

bool AsyncHTTPClient::processResponse() {
    // Process the response
    if (statusCode_ >= 200 && statusCode_ < 300) {
        util::Debug::info("AsyncHTTPClient: Request successful (status " + String(statusCode_) + ")");
        
        // Extract response body (truncated for logging)
        String responsePreview = responseBuffer_.length() > 100 ? 
            responseBuffer_.substring(0, 100) + "..." : 
            responseBuffer_;
        
        // Only log if there's actual data to log
        if (responseBuffer_.length() > 0) {
            util::Debug::info("AsyncHTTPClient: Response: " + responsePreview);
        } else {
            util::Debug::info("AsyncHTTPClient: Empty response body");
        }
        
        lastResponseCode_ = statusCode_;
        successCount_++;
        return true;
    } else {
        util::Debug::warning("AsyncHTTPClient: Request failed with status " + 
                          String(statusCode_ < 0 ? "unknown" : String(statusCode_)));
        lastResponseCode_ = statusCode_;
        errorCount_++;
        return false;
    }
}

void AsyncHTTPClient::closeConnection(bool force) {
    if (client_.connected() && (force || !usePersistentConnection_)) {
        util::Debug::info("AsyncHTTPClient: Closing connection" + 
                       String(force ? " (forced)" : ""));
        client_.stop();
    }
    
    connectionEstablished_ = false;
}

void AsyncHTTPClient::resetState(bool closeConn) {
    if (closeConn) {
        closeConnection(true);
    }
    
    currentState_ = STATE_IDLE;
    requestInProgress_ = false;
    responseBuffer_ = "";
    statusCode_ = -1;
    bytesSent_ = 0;
    
    util::Debug::info("AsyncHTTPClient: Request complete (Success: " + 
                  String(successCount_) + ", Errors: " + 
                  String(errorCount_) + ", Timeouts: " + 
                  String(timeoutCount_) + ")");
}

void AsyncHTTPClient::logTimeoutError() {
    String stateStr = getStateName();
    
    util::Debug::warning("AsyncHTTPClient: Request timed out in state " + stateStr + 
                      " after " + String((millis() - stateStartTime_) / 1000.0, 1) + " seconds");
    
    // Add more details based on state
    if (currentState_ == STATE_CONNECTING) {
        util::Debug::warning("AsyncHTTPClient: Connection to " + 
                          String(serverAddress_) + ":" + String(port_) + " timed out");
    } else if (currentState_ == STATE_SENDING) {
        util::Debug::warning("AsyncHTTPClient: Send timed out after " + 
                         String(bytesSent_) + " bytes sent");
    } else if (currentState_ == STATE_WAITING) {
        util::Debug::warning("AsyncHTTPClient: Timed out waiting for response");
    } else if (currentState_ == STATE_READING) {
        util::Debug::warning("AsyncHTTPClient: Timed out reading response (received " + 
                          String(responseBuffer_.length()) + " bytes)");
        
        // Try to salvage a partial response
        if (statusCode_ >= 200 && statusCode_ < 300 && responseBuffer_.length() > 0) {
            util::Debug::warning("AsyncHTTPClient: Attempting to process partial response");
            processResponse();
        }
    }
}

bool AsyncHTTPClient::ensureConnection() {
    // If we're already connected, just return true
    if (client_.connected() && connectionEstablished_) {
        return true;
    }
    
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
    closeConnection(true);
    responseBuffer_ = "";
    statusCode_ = -1;
    bytesSent_ = 0;
    connectionEstablished_ = false;
    
    // Start connecting
    util::Debug::info("AsyncHTTPClient: Connecting to " + String(serverAddress_) + ":" + String(port_));
    currentState_ = STATE_CONNECTING;
    stateStartTime_ = millis();
    connectionAttempts_++;
    
    // Try to connect (non-blocking)
    if (!client_.connect(serverIP, port_)) {
        // If connect returns false, it might be because it's in progress or failed immediately
        if (client_.connected()) {
            util::Debug::info("AsyncHTTPClient: Connection in progress");
            return true;
        } else {
            util::Debug::error("AsyncHTTPClient: Connection failed immediately");
            currentState_ = STATE_IDLE;
            errorCount_++;
            return false;
        }
    } else {
        util::Debug::info("AsyncHTTPClient: Connected immediately");
        connectionEstablished_ = true;
        return true;
    }
}

bool AsyncHTTPClient::managePersistentConnection() {
    if (!usePersistentConnection_) {
        return true;
    }
    
    // If we're in keepalive state, check for timeout
    if (currentState_ == STATE_KEEPALIVE) {
        uint32_t idleTime = millis() - idleConnectionTime_;
        
        // If we've been idle too long, close the connection
        if (idleTime > MAX_IDLE_CONNECTION_TIME) {
            util::Debug::info("AsyncHTTPClient: Closing idle connection after " + 
                           String(idleTime / 1000) + " seconds");
            closeConnection(true);
            resetState();
            return false;
        }
        
        // Also check if the connection is still actually connected
        if (!client_.connected()) {
            util::Debug::warning("AsyncHTTPClient: Persistent connection lost");
            resetState(true);
            return false;
        }
    }
    
    return true;
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
    
    // Close any existing connection
    closeConnection(true);
    
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

void AsyncHTTPClient::setPersistentConnection(bool enable) {
    // If turning off persistent connection, close any existing connection
    if (usePersistentConnection_ && !enable) {
        closeConnection(true);
    }
    
    usePersistentConnection_ = enable;
    util::Debug::info("AsyncHTTPClient: Persistent connection " + String(enable ? "enabled" : "disabled"));
}

String AsyncHTTPClient::getStateName() const {
    if (currentState_ >= 0 && currentState_ <= STATE_KEEPALIVE) {
        return stateNames[currentState_];
    }
    return "UNKNOWN";
}

} // namespace network
} // namespace baja