#include "network/async_udp_client.hpp"
#include "util/debug_util.hpp"

namespace baja {
namespace network {

AsyncUDPClient::AsyncUDPClient(buffer::RingBuffer<serialization::EncodedMessage, config::PB_MESSAGE_BUFFER_SIZE>& encodedBuffer)
    : encodedBuffer_(encodedBuffer),
      port_(8888),
      isConnected_(false),
      messagesSent_(0),
      oversizedMessages_(0),
      bytesTransferred_(0),
      sendErrors_(0),
      lastStatsTime_(0) {
    
    // Initialize server address to empty string
    serverAddress_[0] = '\0';
    
    util::Debug::info("AsyncUDPClient: Client initialized");
}

AsyncUDPClient::~AsyncUDPClient() {
    // Close UDP connection
    udp_.close();
    util::Debug::info("AsyncUDPClient: Client destroyed");
}

bool AsyncUDPClient::begin(const char* serverAddress, uint16_t port) {
    // Store server configuration
    strncpy(serverAddress_, serverAddress, sizeof(serverAddress_) - 1);
    serverAddress_[sizeof(serverAddress_) - 1] = '\0';
    port_ = port;
    
    util::Debug::info("AsyncUDPClient: Initializing for " + String(serverAddress_) + ":" + String(port_));
    
    // Set up the QNEthernet connection if not already initialized
    if (!isConnected()) {
        if (!initializeNetwork()) {
            util::Debug::error("AsyncUDPClient: Network initialization failed!");
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
            util::Debug::error("AsyncUDPClient: Failed to resolve hostname: " + String(serverAddress_));
            return false;
        }
    }
    
    // Connect to the UDP server
    if (udp_.connect(serverIP, port_)) {
        util::Debug::info("AsyncUDPClient: Connected to UDP server " + String(serverAddress_) + ":" + String(port_));
        isConnected_ = true;
        
        // Set up callback for incoming packets if needed
        udp_.onPacket([](AsyncUDPPacket packet) {
            // In our case, we don't expect responses, but we could process them here
            util::Debug::detail("AsyncUDPClient: Received UDP packet from " + 
                                String(packet.remoteIP()) + ":" + String(packet.remotePort()) +
                                ", length: " + String(packet.length()));
        });
        
        return true;
    } else {
        util::Debug::error("AsyncUDPClient: Failed to connect to UDP server");
        isConnected_ = false;
        return false;
    }
}

bool AsyncUDPClient::initializeNetwork() {
    util::Debug::info("AsyncUDPClient: Initializing QNEthernet...");
    
    // Check if Ethernet is already initialized
    if (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp && Ethernet.localIP() != IPAddress(0, 0, 0, 0)) {
        util::Debug::info("AsyncUDPClient: Network already initialized with IP: " + ipToString(Ethernet.localIP()));
        return true;
    }
    
    
    #if USING_DHCP
        util::Debug::info("AsyncUDPClient: Using DHCP...");
        if (!Ethernet.begin()) {
            util::Debug::error("AsyncUDPClient: DHCP configuration failed");
            return false;
        }
        
        // Wait for DHCP to complete
        if (!Ethernet.waitForLocalIP(10000)) {
            util::Debug::error("AsyncUDPClient: DHCP timeout");
            return false;
        }
    #else
        util::Debug::info("AsyncUDPClient: Using static IP...");
        
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
    util::Debug::info("AsyncUDPClient: Waiting for network link...");
    while (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp && millis() - startTime < 5000) {
        delay(100);
    }
    
    if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
        util::Debug::error("AsyncUDPClient: Ethernet link not established");
        return false;
    }
    
    // Get and log the IP address
    IPAddress localIP = Ethernet.localIP();
    util::Debug::info("AsyncUDPClient: Connected with IP: " + ipToString(localIP));
    
    return true;
}

String AsyncUDPClient::ipToString(IPAddress ip) {
    return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

size_t AsyncUDPClient::process(size_t maxMessages, uint32_t maxTimeUs) {
    // First check if network is connected
    if (!checkConnection()) {
        util::Debug::warning("AsyncUDPClient: Network connection is down");
        return 0;
    }
    
    // Get number of available encoded messages
    size_t availableMessages = encodedBuffer_.available();
    if (availableMessages == 0) {
        return 0;
    }
    
    // Set max messages if not specified
    if (maxMessages == 0 || maxMessages > availableMessages) {
        maxMessages = availableMessages;
    }
    
    // Set start time for timing constraint
    uint32_t startTime = micros();
    size_t processedCount = 0;


    // Process messages up to max count or time limit
    for (size_t i = 0; i < maxMessages; i++) {
        // Check if we've exceeded the max processing time
        if (maxTimeUs > 0 && (micros() - startTime) > maxTimeUs) {
            break;
        }
        


        // Read an encoded message from the buffer
        serialization::EncodedMessage encodedMsg;
        if (!encodedBuffer_.read(encodedMsg)) {
            util::Debug::warning("AsyncUDPClient: Failed to read encoded message from buffer");
            break;
        }
        yield();
        
        // Send the message
        if (sendMessage(encodedMsg)) {
            processedCount++;
        }
    }
    
    // Log stats periodically
    logStats();
    
    return processedCount;
}

bool AsyncUDPClient::sendMessage(const serialization::EncodedMessage& encodedMsg) {
    // Check if message is too large for UDP
    if (encodedMsg.size > MAX_UDP_PAYLOAD) {
        util::Debug::warning("AsyncUDPClient: Message size (" + String(encodedMsg.size) + 
                           " bytes) exceeds UDP payload limit (" + String(MAX_UDP_PAYLOAD) + " bytes)");
        oversizedMessages_++;
        return false;
    }

    // Send the raw encoded data directly
    if (!udp_.write(encodedMsg.buffer, encodedMsg.size)) {
        util::Debug::warning("AsyncUDPClient: Failed to send UDP message");
        sendErrors_++;
        return false;
    }

    // Update statistics
    messagesSent_++;
    bytesTransferred_ += encodedMsg.size;
    
    // Log detailed message only at debug level
    util::Debug::detail("AsyncUDPClient: Sent message #" + String(messagesSent_) + 
                      " of " + String(encodedMsg.size) + " bytes");
    
    return true;
}

bool AsyncUDPClient::isConnected() const {
    return isConnected_ && checkConnection();
}

bool AsyncUDPClient::checkConnection() {
    // Check if Ethernet link is up
    bool linkUp = (Ethernet.linkStatus() == LinkStatus_kLinkStatusUp);
    
    // Check if we have a valid IP address
    IPAddress ip = Ethernet.localIP();
    bool validIP = !(ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0);
    
    return linkUp && validIP;
}

void AsyncUDPClient::logStats() {
    uint32_t currentTime = millis();
    
    // Log stats every 10 seconds
    if (currentTime - lastStatsTime_ >= 10000) {
        lastStatsTime_ = currentTime;
        
        float bytesKB = bytesTransferred_ / 1024.0f;
        float avgMsgSize = messagesSent_ > 0 ? (float)bytesTransferred_ / messagesSent_ : 0;
        
        util::Debug::info("AsyncUDPClient: Stats - Sent: " + String(messagesSent_) + 
                        " messages (" + String(bytesKB, 1) + " KB), Avg size: " + 
                        String(avgMsgSize, 1) + " bytes");
        
        if (oversizedMessages_ > 0) {
            util::Debug::warning("AsyncUDPClient: Dropped " + String(oversizedMessages_) + 
                              " oversized messages");
        }
        
        if (sendErrors_ > 0) {
            util::Debug::warning("AsyncUDPClient: Encountered " + String(sendErrors_) + 
                              " send errors");
        }
    }
}

} // namespace network
} // namespace baja