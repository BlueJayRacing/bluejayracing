/****************************************************************************************************************************
  HighSpeedPOST.ino - High performance HTTP client for Teensy41 QNEthernet
  
  For Teensy41 with QNEthernet
  
  This example implements:
  - Multiple simultaneous connections for higher throughput (10 concurrent connections)
  - 1000 byte message size
  - HTTP keep-alive for connection reuse
  - Coordinated scheduling of connections to reduce errors
  - Detailed timing and error metrics
  - Improved stability with watchdog, memory management, and connection recovery
 *****************************************************************************************************************************/

 #include "defines.h"
 // Built-in watchdog for Teensy 4.1
 #include <Arduino.h>
 
 // Server configuration
 const char POST_ServerAddress[] = "192.168.20.3";  // Change to your server IP
 const uint16_t POST_ServerPort = 9365;            // Change to your server port
 const char POST_Endpoint[] = "/api/data";         // Change to your endpoint
   
 // Message and connection configuration
 #define MESSAGE_SIZE 1000                    // 1000 byte message size
 #define CONNECTION_POOL_SIZE 10               // Number of simultaneous connections to use
 #define CONNECTION_INTERVAL_MS 80            // Interval between scheduled connections (staggered)
 #define KEEP_ALIVE_TIMEOUT_MS 30000         // Keep-alive timeout (30 seconds)
 #define CONNECTION_TIMEOUT_MS 10000         // Connection timeout (10 seconds)
 #define MAX_CONSECUTIVE_ERRORS 3            // Max consecutive errors before cooling down a connection
 
 // HTTP client configuration
 #define ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN_TARGET      "AsyncHTTPRequest_Teensy41 v1.10.0"
 #define ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN             1010000
   
 // Debug level (0-4)
 #define ASYNC_HTTP_DEBUG_PORT     Serial
 #define _ASYNC_HTTP_LOGLEVEL_     1
   
 // Include AsyncHTTPRequest
 #include <AsyncHTTPRequest_Teensy41.h>
 #include <Ticker.h>
 
 // Define built-in watchdog functions for Teensy 4.1
 // No external library required
 void enableWatchdog(unsigned timeout_ms = 5000) {
   // Configure the watchdog timer using direct register access
   // This works on Teensy 4.1 without additional libraries
   noInterrupts();
   
   WDOG1_WMCR = 0;                      // Clear the WDOG_WMCR register
   WDOG1_WCR |= WDOG_WCR_SRS;           // System reset enabled
   WDOG1_WCR |= WDOG_WCR_WT(timeout_ms / 500); // Set timeout value
   WDOG1_WCR |= WDOG_WCR_WDE;           // Enable watchdog
   
   interrupts();
 }
 
 void resetWatchdog() {
   // Service the watchdog by writing the WDOG service sequence
   WDOG1_WSR = 0x5555;
   WDOG1_WSR = 0xAAAA;
 }
 
 // System health monitoring
 uint32_t freeRamLow = UINT32_MAX;
 uint32_t loopTimeMax = 0;
 uint32_t lastLoopStartTime = 0;
 
 // Function to get free RAM on Teensy 4.1
 uint32_t getFreeRAM() {
   extern unsigned long _heap_start;
   extern unsigned long _heap_end;
   extern char *__brkval;
   
   uint32_t free_memory;
   
   if (__brkval == 0)
     free_memory = ((uint32_t)&_heap_end - (uint32_t)&_heap_start);
   else
     free_memory = ((uint32_t)&_heap_end - (uint32_t)__brkval);
     
   return free_memory;
 }
 
 // State machine states
 enum HttpClientState {
   STATE_IDLE,
   STATE_CONNECTING,
   STATE_SENDING,
   STATE_WAITING,
   STATE_READING,
   STATE_PROCESSING,
   STATE_CLOSING,
   STATE_KEEPALIVE,
   STATE_COOLDOWN   // State for connections that need to cool down after errors
 };
 
 // Forward declaration for getStateName function 
 const char* getStateName(HttpClientState state);
 
 // Connection structure to track each connection's state
 struct Connection {
   EthernetClient client;
   HttpClientState state = STATE_IDLE;
   uint32_t stateStartTime = 0;
   uint32_t lastSendTime = 0;
   uint32_t lastActivityTime = 0;
   uint32_t messagesSent = 0;
   uint32_t bytesSent = 0;
   uint32_t errCount = 0;
   uint32_t consecutiveErrors = 0;
   uint32_t cooldownUntil = 0;    // Timestamp when cooldown ends
   
   // Timing metrics
   uint32_t lastConnectTime = 0;  // How long to establish connection
   uint32_t lastSendDuration = 0; // How long to send data
   uint32_t totalConnectTime = 0;
   uint32_t totalSendTime = 0;
   uint32_t requestCount = 0;     // Successful requests for timing calculations
   
   bool inUse = false;
   int connectionIndex = 0;       // Store connection index for debugging
 
   // Reset connection state
   void reset() {
     if (client.connected()) {
       client.stop();
     }
     state = STATE_IDLE;
     inUse = false;
     consecutiveErrors = 0;
     cooldownUntil = 0;
   }
   
   // Calculate average connect time
   float avgConnectTime() {
     return (requestCount > 0) ? (float)totalConnectTime / requestCount : 0;
   }
   
   // Calculate average send time
   float avgSendTime() {
     return (requestCount > 0) ? (float)totalSendTime / requestCount : 0;
   }
   
   // Verify connection state is valid
   bool verifyConnectionState() {
     // Check if the connection is in a valid state
     if (state != STATE_IDLE && state != STATE_COOLDOWN) {
       // For any active state, check if the client is actually connected
       if (state != STATE_CONNECTING && !client.connected()) {
         #if _ASYNC_HTTP_LOGLEVEL_ > 0
         Serial.print("Connection #");
         Serial.print(connectionIndex);
         Serial.print(" in state ");
         Serial.print(getStateName(state));
         Serial.println(" but client is disconnected");
         #endif
         
         // Reset the connection
         reset();
         return false;
       }
     }
     
     // Check for stale states
     if ((state == STATE_CONNECTING || state == STATE_SENDING) && 
         (millis() - stateStartTime > CONNECTION_TIMEOUT_MS / 2)) {
       // Connection is taking too long, verify it's still progressing
       if (!client.connected()) {
         #if _ASYNC_HTTP_LOGLEVEL_ > 0
         Serial.print("Connection #");
         Serial.print(connectionIndex);
         Serial.println(" appears stalled, resetting");
         #endif
         
         reset();
         return false;
       }
     }
     
     return true;
   }
 };
 
 // Client class that manages a pool of connections
 class HighSpeedClient {
 private:
   // Connection pool
   Connection connections[CONNECTION_POOL_SIZE];
   IPAddress serverIP;
   uint16_t serverPort;
   const char* endpoint;
   
   // Using static buffer to avoid stack issues
   static char messageBuffer[MESSAGE_SIZE + 512]; // Extra space for HTTP headers
   
   // Scheduling
   uint32_t nextScheduledTime[CONNECTION_POOL_SIZE];
   uint32_t schedulingOffset = 0;
   
   // Statistics
   uint32_t totalMessagesSent = 0;
   uint32_t totalBytesSent = 0;
   uint32_t totalErrors = 0;
   uint32_t lastStatsTime = 0;
   uint32_t totalConnectTime = 0;
   uint32_t totalSendTime = 0;
   uint32_t successfulRequests = 0;
   uint32_t lastNetworkCheckTime = 0;
   
 public:
   HighSpeedClient() {}
   
   // Initialize with server details
   bool begin(const char* serverAddress, uint16_t port, const char* serverEndpoint) {
     endpoint = serverEndpoint;
     serverPort = port;
     
     // Resolve server IP
     if (!Ethernet.hostByName(serverAddress, serverIP)) {
       // Try parsing as direct IP address
       int a, b, c, d;
       if (sscanf(serverAddress, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
         serverIP = IPAddress(a, b, c, d);
       } else {
         Serial.println("Failed to resolve hostname");
         return false;
       }
     }
     
     Serial.print("Server IP: ");
     Serial.println(serverIP);
     Serial.print("Using ");
     Serial.print(CONNECTION_POOL_SIZE);
     Serial.println(" simultaneous connections");
     
     // Initialize all connections and schedule them
     for (int i = 0; i < CONNECTION_POOL_SIZE; i++) {
       connections[i].reset();
       connections[i].connectionIndex = i;
       
       // Stagger initial connection attempts
       nextScheduledTime[i] = millis() + (i * (CONNECTION_INTERVAL_MS / CONNECTION_POOL_SIZE));
     }
     
     return true;
   }
   
   // Check if Ethernet connection is still active and recover if needed
   bool checkEthernetConnection() {
     // Check if Ethernet connection is still active
     if (!Ethernet.linkStatus()) {
       Serial.println("Ethernet link down, attempting recovery");
       
       // Close all existing connections
       closeAll();
       
       // Try to reinitialize Ethernet
       #if USING_DHCP
         Ethernet.begin();
       #else
         Ethernet.begin(myIP, myNetmask, myGW);
         Ethernet.setDNSServerIP(mydnsServer);
       #endif
       
       // Wait for connection to be established
       if (!Ethernet.waitForLocalIP(5000)) {
         Serial.println("Failed to recover Ethernet connection");
         return false;
       }
       
       Serial.print("Ethernet recovered! IP: ");
       Serial.println(Ethernet.localIP());
       return true;
     }
     
     return true; // Link is up
   }
   
   // Periodic health check for the network connection
   void periodicCheck() {
     uint32_t now = millis();
     
     // Check network health every 30 seconds
     if (now - lastNetworkCheckTime >= 30000) {
       if (!checkEthernetConnection()) {
         // If recovery failed, try again sooner
         lastNetworkCheckTime = now - 25000;
       } else {
         lastNetworkCheckTime = now;
       }
     }
   }
 
   // Check if it's time to schedule any connections
   void checkSchedule() {
     uint32_t now = millis();
     
     // Check each connection to see if it's time to schedule it
     for (int i = 0; i < CONNECTION_POOL_SIZE; i++) {
       Connection* conn = &connections[i];
       
       // Skip if connection is already in use or in cooldown
       if (conn->inUse || conn->state == STATE_COOLDOWN) {
         continue;
       }
       
       // Check if this connection is scheduled to run now
       if (conn->state == STATE_IDLE || conn->state == STATE_KEEPALIVE) {
         if (now >= nextScheduledTime[i]) {
           // It's time to schedule this connection
           if (sendMessage(conn)) {
             // Update next scheduling time
             nextScheduledTime[i] = now + CONNECTION_INTERVAL_MS;
           } else {
             // If failed to schedule, try again shortly
             nextScheduledTime[i] = now + (CONNECTION_INTERVAL_MS / 2);
           }
         }
       }
     }
   }
   
   // Send a message using a specific connection
   bool sendMessage(Connection* conn) {
     if (!conn || conn->inUse) {
       return false;
     }
     
     uint32_t now = millis();
     
     // Check if connection is in cooldown
     if (conn->state == STATE_COOLDOWN) {
       if (now < conn->cooldownUntil) {
         return false;
       } else {
         // Cooldown period is over
         conn->state = STATE_IDLE;
       }
     }
     
     // Generate message content
     generateRandomMessage();
     
     // If we're in keep-alive state, move directly to sending
     if (conn->state == STATE_KEEPALIVE) {
       conn->state = STATE_SENDING;
       conn->stateStartTime = now;
       conn->inUse = true;
       return true;
     }
     
     // Mark start time for connection timing
     uint32_t connectStartTime = now;
     
     // Not connected yet, establish connection
     if (!conn->client.connect(serverIP, serverPort)) {
       #if _ASYNC_HTTP_LOGLEVEL_ > 1
       Serial.print("Connection failed for #");
       Serial.println(conn->connectionIndex);
       #endif
       
       conn->errCount++;
       conn->consecutiveErrors++;
       totalErrors++;
       
       // Check if we've had too many consecutive errors
       if (conn->consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
         #if _ASYNC_HTTP_LOGLEVEL_ > 0
         Serial.print("Connection #");
         Serial.print(conn->connectionIndex);
         Serial.println(" entering cooldown after repeated errors");
         #endif
         
         // Calculate cooldown time based on consecutive errors (exponential backoff)
         uint32_t cooldownTime = 500 * (1 << min(conn->consecutiveErrors - MAX_CONSECUTIVE_ERRORS, 7));
         conn->state = STATE_COOLDOWN;
         conn->cooldownUntil = now + cooldownTime;
       } else {
         conn->reset();
       }
       
       return false;
     }
     
     // Track connection time
     conn->lastConnectTime = millis() - connectStartTime;
     conn->totalConnectTime += conn->lastConnectTime;
     totalConnectTime += conn->lastConnectTime;
     
     // Connection in progress or established
     conn->state = STATE_CONNECTING;
     conn->stateStartTime = now;
     conn->inUse = true;
     conn->lastSendTime = now;
     return true;
   }
   
   // Process all connections (call frequently from loop)
   void process() {
     uint32_t now = millis();
     
     // Check network health periodically
     periodicCheck();
     
     // Process each connection
     for (int i = 0; i < CONNECTION_POOL_SIZE; i++) {
       Connection* conn = &connections[i];
       
       // Skip connections that aren't in use and are idle
       if (!conn->inUse && (conn->state == STATE_IDLE || conn->state == STATE_COOLDOWN)) {
         continue;
       }
       
       // Verify connection state is valid before processing
       if (!conn->verifyConnectionState()) {
         continue; // Skip processing if connection was reset
       }
       
       // Handle state timeouts
       if (conn->state != STATE_IDLE && conn->state != STATE_KEEPALIVE && conn->state != STATE_COOLDOWN) {
         // Check for timeout
         if (now - conn->stateStartTime > CONNECTION_TIMEOUT_MS) {
           #if _ASYNC_HTTP_LOGLEVEL_ > 1
           Serial.print("Timeout in state: ");
           Serial.print(getStateName(conn->state));
           Serial.print(" for connection #");
           Serial.println(conn->connectionIndex);
           #endif
           
           conn->errCount++;
           conn->consecutiveErrors++;
           totalErrors++;
           
           // Handle too many consecutive errors
           if (conn->consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
             #if _ASYNC_HTTP_LOGLEVEL_ > 0
             Serial.print("Connection #");
             Serial.print(conn->connectionIndex);
             Serial.println(" entering cooldown after timeout");
             #endif
             
             uint32_t cooldownTime = 500 * (1 << min(conn->consecutiveErrors - MAX_CONSECUTIVE_ERRORS, 7));
             conn->state = STATE_COOLDOWN;
             conn->cooldownUntil = now + cooldownTime;
             conn->inUse = false;
           } else {
             conn->reset();
           }
           
           continue;
         }
       }
       
       // Check if keep-alive connection has timed out
       if (conn->state == STATE_KEEPALIVE) {
         if (now - conn->lastActivityTime > KEEP_ALIVE_TIMEOUT_MS) {
           #if _ASYNC_HTTP_LOGLEVEL_ > 1
           Serial.print("Keep-alive timeout for connection #");
           Serial.println(conn->connectionIndex);
           #endif
           conn->reset();
           continue;
         }
       }
       
       // Process state machine
       switch (conn->state) {
         case STATE_CONNECTING:
           if (conn->client.connected()) {
             #if _ASYNC_HTTP_LOGLEVEL_ > 2
             Serial.print("Connection #");
             Serial.print(conn->connectionIndex);
             Serial.println(" established, sending request");
             #endif
             conn->state = STATE_SENDING;
             conn->stateStartTime = now;
           }
           break;
         
         case STATE_SENDING: {
           // Mark start time for send timing
           uint32_t sendStartTime = now;
           
           if (sendHttpRequest(conn)) {
             // Fire-and-forget mode: don't wait for response, move directly to keep-alive
             conn->state = STATE_KEEPALIVE;
             conn->lastActivityTime = now;
             conn->messagesSent++;
             conn->lastSendTime = now;
             
             // Track send time
             conn->lastSendDuration = millis() - sendStartTime;
             conn->totalSendTime += conn->lastSendDuration;
             totalSendTime += conn->lastSendDuration;
             
             // Update global stats
             totalMessagesSent++;
             totalBytesSent += MESSAGE_SIZE;
             conn->requestCount++;
             successfulRequests++;
             
             // Reset consecutive errors since this was successful
             conn->consecutiveErrors = 0;
             
             // Make connection available again
             conn->inUse = false;
           } else {
             // Failed to send
             conn->errCount++;
             conn->consecutiveErrors++;
             totalErrors++;
             
             // Handle too many consecutive errors
             if (conn->consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
               #if _ASYNC_HTTP_LOGLEVEL_ > 0
               Serial.print("Connection #");
               Serial.print(conn->connectionIndex);
               Serial.println(" entering cooldown after send failure");
               #endif
               
               uint32_t cooldownTime = 500 * (1 << min(conn->consecutiveErrors - MAX_CONSECUTIVE_ERRORS, 7));
               conn->state = STATE_COOLDOWN;
               conn->cooldownUntil = now + cooldownTime;
               conn->inUse = false;
             } else {
               conn->reset();
             }
           }
           break;
         }
            
         case STATE_KEEPALIVE:
           // Check if connection is still active
           if (!conn->client.connected()) {
             #if _ASYNC_HTTP_LOGLEVEL_ > 1
             Serial.print("Keep-alive connection #");
             Serial.print(conn->connectionIndex);
             Serial.println(" lost");
             #endif
             conn->reset();
           }
           break;
           
         case STATE_COOLDOWN:
           // Check if cooldown period is over
           if (now >= conn->cooldownUntil) {
             #if _ASYNC_HTTP_LOGLEVEL_ > 1
             Serial.print("Connection #");
             Serial.print(conn->connectionIndex);
             Serial.println(" cooldown complete");
             #endif
             conn->state = STATE_IDLE;
           }
           break;
            
         default:
           // Nothing to do in other states
           break;
       }
     }
     
     // Print stats every 2 seconds
     if (now - lastStatsTime >= 2000) {
       printStats();
       lastStatsTime = now;
     }
   }
   
   // Close all connections
   void closeAll() {
     for (int i = 0; i < CONNECTION_POOL_SIZE; i++) {
       connections[i].reset();
     }
     Serial.println("All connections closed");
   }
   
   // Get total message count
   uint32_t getMessageCount() {
     return totalMessagesSent;
   }
   
   // Get total error count
   uint32_t getErrorCount() {
     return totalErrors;
   }
   
 private:
   // Helper functions
   void generateRandomMessage() {
     // Create payload of exactly 1000 bytes
     int contentLength = MESSAGE_SIZE;
     int headerSize = 0;
     
     // HTTP headers
     headerSize += sprintf(messageBuffer, "POST %s HTTP/1.1\r\n", endpoint);
     headerSize += sprintf(messageBuffer + headerSize, "Host: %s\r\n", POST_ServerAddress);
     headerSize += sprintf(messageBuffer + headerSize, "Content-Type: application/json\r\n");
     headerSize += sprintf(messageBuffer + headerSize, "Connection: keep-alive\r\n");
     headerSize += sprintf(messageBuffer + headerSize, "Content-Length: %d\r\n\r\n", contentLength);
     
     // Generate JSON content
     headerSize += sprintf(messageBuffer + headerSize, "{\"device\":\"Teensy41\",\"timestamp\":%lu,\"data\":\"", millis());
     
     // Fill the rest with random hex data up to exactly 1000 bytes
     int randomDataSize = contentLength - (headerSize + 50); // Allow space for closing tags
     for (int i = 0; i < randomDataSize; i++) {
       char hexChar = "0123456789ABCDEF"[random(16)];
       messageBuffer[headerSize + i] = hexChar;
     }
     
     // Closing JSON structure
     sprintf(messageBuffer + headerSize + randomDataSize, "\",\"id\":%lu}", totalMessagesSent);
   }
   
   bool sendHttpRequest(Connection* conn) {
     // Send the complete HTTP request with payload
     size_t len = strlen(messageBuffer);
     size_t written = conn->client.write(messageBuffer, len);
     
     if (written == len) {
       conn->bytesSent += written;
       return true;
     }
     
     #if _ASYNC_HTTP_LOGLEVEL_ > 1
     Serial.print("Send error for connection #");
     Serial.print(conn->connectionIndex);
     Serial.print(": wrote ");
     Serial.print(written);
     Serial.print(" of ");
     Serial.println(len);
     #endif
     return false;
   }
   
   void printStats() {
     // Calculate message rate
     static uint32_t lastMsgCount = 0;
     static uint32_t lastBytesCount = 0;
     static uint32_t lastSuccessCount = 0;
     
     uint32_t msgDelta = totalMessagesSent - lastMsgCount;
     uint32_t bytesDelta = totalBytesSent - lastBytesCount;
     uint32_t successDelta = successfulRequests - lastSuccessCount;
     
     float rate = msgDelta / 2.0f; // messages per second over 2-second interval
     float dataRate = bytesDelta / 2.0f / 1024.0f; // KB per second
     
     // Calculate average request times
     float avgConnectMs = (successfulRequests > 0) ? (float)totalConnectTime / successfulRequests : 0;
     float avgSendMs = (successfulRequests > 0) ? (float)totalSendTime / successfulRequests : 0;
     float avgTotalMs = avgConnectMs + avgSendMs;
     
     // Calculate success rate
     float successRate = (totalMessagesSent + totalErrors > 0) ? 
                        (float)totalMessagesSent / (totalMessagesSent + totalErrors) * 100.0f : 0;
     
     // Count active connections
     int activeCount = 0;
     int keepAliveCount = 0;
     int cooldownCount = 0;
     
     for (int i = 0; i < CONNECTION_POOL_SIZE; i++) {
       if (connections[i].inUse || connections[i].state != STATE_IDLE) {
         activeCount++;
         
         if (connections[i].state == STATE_KEEPALIVE) {
           keepAliveCount++;
         }
         if (connections[i].state == STATE_COOLDOWN) {
           cooldownCount++;
         }
       }
     }
     
     Serial.println("\n---- HTTP Client Statistics ----");
     Serial.print("Rate: ");
     Serial.print(rate, 1);
     Serial.print(" req/s (");
     Serial.print(dataRate, 1);
     Serial.println(" KB/s)");
     
     Serial.print("Total: ");
     Serial.print(totalMessagesSent);
     Serial.print(" requests, ");
     Serial.print(totalBytesSent / 1024);
     Serial.print(" KB, Errors: ");
     Serial.print(totalErrors);
     Serial.print(" (Success rate: ");
     Serial.print(successRate, 1);
     Serial.println("%)");
     
     Serial.print("Avg times - Connect: ");
     Serial.print(avgConnectMs, 1);
     Serial.print("ms, Send: ");
     Serial.print(avgSendMs, 1);
     Serial.print("ms, Total: ");
     Serial.print(avgTotalMs, 1);
     Serial.println("ms");
     
     Serial.print("Connections: ");
     Serial.print(activeCount);
     Serial.print(" active (");
     Serial.print(keepAliveCount);
     Serial.print(" keep-alive, ");
     Serial.print(cooldownCount);
     Serial.print(" cooldown) of ");
     Serial.println(CONNECTION_POOL_SIZE);
     
     // Connection details
     #if _ASYNC_HTTP_LOGLEVEL_ > 2
     Serial.println("Connection Details:");
     for (int i = 0; i < CONNECTION_POOL_SIZE; i++) {
       Serial.print("  #");
       Serial.print(i);
       Serial.print(": ");
       Serial.print(getStateName(connections[i].state));
       Serial.print(", Sent: ");
       Serial.print(connections[i].messagesSent);
       Serial.print(", Errors: ");
       Serial.print(connections[i].errCount);
       Serial.print(", Avg connect: ");
       Serial.print(connections[i].avgConnectTime(), 1);
       Serial.println("ms");
     }
     #endif
     
     // Add system health metrics to stats
     Serial.print("System Health - Free RAM: ");
     Serial.print(getFreeRAM());
     Serial.print(" bytes (low: ");
     Serial.print(freeRamLow);
     Serial.print(" bytes), Max loop time: ");
     Serial.print(loopTimeMax);
     Serial.println("ms");
     
     // Update counters for next calculation
     lastMsgCount = totalMessagesSent;
     lastBytesCount = totalBytesSent;
     lastSuccessCount = successfulRequests;
   }
 };
 
 // Static buffer declaration
 char HighSpeedClient::messageBuffer[MESSAGE_SIZE + 512];
 
 // Global client
 HighSpeedClient httpClient;
 
 // Helper function for getting state names (moved outside the class for easier access)
 const char* getStateName(HttpClientState state) {
   switch (state) {
     case STATE_IDLE: return "IDLE";
     case STATE_CONNECTING: return "CONNECTING";
     case STATE_SENDING: return "SENDING";
     case STATE_WAITING: return "WAITING";
     case STATE_READING: return "READING";
     case STATE_PROCESSING: return "PROCESSING";
     case STATE_CLOSING: return "CLOSING";
     case STATE_KEEPALIVE: return "KEEPALIVE";
     case STATE_COOLDOWN: return "COOLDOWN";
     default: return "UNKNOWN";
   }
 }
 
 void setup() {
   Serial.begin(115200);
   while (!Serial && millis() < 5000);
 
   Serial.print("\nStarting HighSpeedPOST with multiple connections on ");
   Serial.println(BOARD_NAME);
   Serial.println(ASYNC_HTTP_REQUEST_TEENSY41_VERSION);
 
   // Initialize Ethernet
   #if USING_DHCP
     Serial.print("Initialize Ethernet using DHCP => ");
     Ethernet.begin();
   #else
     Serial.print("Initialize Ethernet using static IP => ");
     Ethernet.begin(myIP, myNetmask, myGW);
     Ethernet.setDNSServerIP(mydnsServer);
   #endif
 
   if (!Ethernet.waitForLocalIP(5000)) {
     Serial.println(F("Failed to configure Ethernet"));
     if (!Ethernet.linkStatus()) {
       Serial.println(F("Ethernet cable is not connected."));
     }
     while (true) {
       delay(1);
     }
   } else {
     Serial.print(F("Connected! IP address:"));
     Serial.println(Ethernet.localIP());
   }
 
   // Wait for network to fully initialize
   delay(1000);
   
   // Seed random number generator
   randomSeed(micros());
   
   // Initialize client
   httpClient.begin(POST_ServerAddress, POST_ServerPort, POST_Endpoint);
   
   // Configure and start the builtin watchdog
   enableWatchdog(5000);  // 5 second timeout
   resetWatchdog();
   
   Serial.println("High-speed POST client with scheduled connections started");
   Serial.print("Sending ");
   Serial.print(MESSAGE_SIZE);
   Serial.print(" byte messages using ");
   Serial.print(CONNECTION_POOL_SIZE);
   Serial.println(" concurrent connections");
   
   // Initialize tracking variables
   lastLoopStartTime = millis();
 }
 
 void loop() {
   // Reset watchdog at the start of each loop
   resetWatchdog();
   
   // Track loop timing
   uint32_t currentTime = millis();
   uint32_t loopTime = currentTime - lastLoopStartTime;
   lastLoopStartTime = currentTime;
   
   // Track longest loop time
   if (loopTime > loopTimeMax && lastLoopStartTime > 10000) { // Ignore startup
     loopTimeMax = loopTime;
     
     #if _ASYNC_HTTP_LOGLEVEL_ > 0
     if (loopTimeMax > 500) { // Only log significant delays
       Serial.print("Warning: Long loop time detected: ");
       Serial.print(loopTimeMax);
       Serial.println("ms");
     }
     #endif
   }
   
   // Monitor free RAM
   uint32_t freeRam = getFreeRAM();
   if (freeRam < freeRamLow) {
     freeRamLow = freeRam;
     
     #if _ASYNC_HTTP_LOGLEVEL_ > 0
     Serial.print("Free RAM low mark: ");
     Serial.println(freeRamLow);
     #endif
   }
   
   // Check for critically low memory
   if (freeRam < 10000) { // Adjust this threshold based on your application
     Serial.println("WARNING: Memory critically low, resetting all connections");
     httpClient.closeAll();
     delay(1000); // Give system time to recover
   }
 
   // Check for scheduling connections
   httpClient.checkSchedule();
   
   // Process the HTTP client connections
   httpClient.process();
   
   // Reset watchdog again at the end of loop for safety
   resetWatchdog();
 }