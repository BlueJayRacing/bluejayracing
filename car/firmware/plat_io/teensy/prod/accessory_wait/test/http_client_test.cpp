#include <Arduino.h>
#include <QNEthernet.h>
#include "config/defines.h"

using namespace qindesign::network;

// Server configuration
const char* SERVER_ADDRESS = "192.168.20.3";
const uint16_t SERVER_PORT = 9365;
const char* SERVER_ENDPOINT = "/";

// Request timing
const uint32_t REQUEST_INTERVAL = 15000;  // 15 seconds

// Statistics
uint32_t requestCount = 0;
uint32_t successCount = 0;
uint32_t errorCount = 0;
uint32_t lastRequestTime = 0;

// Function prototypes
bool initializeNetwork();
bool sendDirectHttpRequest();

void setup() {
  // Initialize serial and wait for it to be ready
  Serial.begin(115200);
  uint32_t startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    // Wait up to 3 seconds for Serial
  }
  
  Serial.println("\n\nTeensy 4.1 Direct HTTP Client Test");
  Serial.println("===================================");
  
  // Initialize Ethernet
  if (!initializeNetwork()) {
    Serial.println("Network initialization failed! Halting.");
    while (1) { delay(1000); }
  }
  
  Serial.println("Setup complete. Will send HTTP requests every 15 seconds.");
  lastRequestTime = millis() - REQUEST_INTERVAL + 5000;  // First request after 5 seconds
}

void loop() {
  uint32_t currentTime = millis();
  
  // Check if it's time to send a request
  if (currentTime - lastRequestTime >= REQUEST_INTERVAL) {
    lastRequestTime = currentTime;
    sendDirectHttpRequest();
  }
  
  // Print stats periodically
  static uint32_t lastStatsTime = 0;
  if (currentTime - lastStatsTime >= 30000) { // Every 30 seconds
    lastStatsTime = currentTime;
    
    Serial.println("\n--- HTTP Client Statistics ---");
    Serial.println("Total requests: " + String(requestCount));
    Serial.println("Successful responses: " + String(successCount));
    Serial.println("Errors: " + String(errorCount));
    Serial.println("Network status: " + String(Ethernet.linkStatus() == LinkStatus_kLinkStatusUp ? "Connected" : "Disconnected"));
    Serial.println("IP Address: " + String(Ethernet.localIP()[0]) + "." + 
                                   String(Ethernet.localIP()[1]) + "." + 
                                   String(Ethernet.localIP()[2]) + "." + 
                                   String(Ethernet.localIP()[3]));
    Serial.println("-----------------------------\n");
  }
  
  // Small delay to prevent tight loop
  delay(10);
}

// Network initialization
bool initializeNetwork() {
  Serial.println("\nInitializing network...");
  
  // End any existing connections
  Ethernet.end();
  delay(500);
  
  // Initialize with static IP
  IPAddress staticIP(192, 168, 20, 13);
  IPAddress gateway(192, 168, 20, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);
  
  // Begin with static IP
  Ethernet.begin(staticIP, subnet, gateway);
  Ethernet.setDNSServerIP(dns);
  
  // Wait for link to come up
  Serial.println("Waiting for link...");
  uint32_t startTime = millis();
  while (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp && millis() - startTime < 8000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  // Check if link is up
  if (Ethernet.linkStatus() != LinkStatus_kLinkStatusUp) {
    Serial.println("Failed to establish Ethernet link!");
    return false;
  }
  
  // Get and log the IP address
  IPAddress ip = Ethernet.localIP();
  Serial.println("Network initialized with IP: " + 
                String(ip[0]) + "." + 
                String(ip[1]) + "." + 
                String(ip[2]) + "." + 
                String(ip[3]));
  
  return true;
}

// Send HTTP request using direct EthernetClient
bool sendDirectHttpRequest() {
  requestCount++;
  
  Serial.println("\n[" + String(millis()) + "] Sending direct HTTP request #" + String(requestCount));
  
  // Create simple JSON payload
  String jsonPayload = "{\"device\":\"Teensy41\",";
  jsonPayload += "\"timestamp\":" + String(millis()) + ",";
  jsonPayload += "\"sequence\":" + String(requestCount) + ",";
  jsonPayload += "\"samples\":[";
  
  // Add 3 sample data points
  for (int i = 0; i < 3; i++) {
    if (i > 0) jsonPayload += ",";
    jsonPayload += "{\"ts\":" + String(millis() + i) + ",";
    jsonPayload += "\"ch\":" + String(i) + ",";
    jsonPayload += "\"val\":" + String(random(0, 4096)) + "}";
  }
  
  jsonPayload += "]}";
  
  // Parse the server IP address
  IPAddress serverIP;
  int a, b, c, d;
  if (sscanf(SERVER_ADDRESS, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
    serverIP = IPAddress(a, b, c, d);
  } else {
    int result = Ethernet.hostByName(SERVER_ADDRESS, serverIP);
    if (result <= 0) {
      Serial.println("Failed to resolve hostname: " + String(SERVER_ADDRESS));
      errorCount++;
      return false;
    }
  }
  
  // Create a TCP client
  EthernetClient client;
  
  // Connect to the server
  Serial.println("Connecting to " + String(SERVER_ADDRESS) + ":" + String(SERVER_PORT) + "...");
  if (!client.connect(serverIP, SERVER_PORT)) {
    Serial.println("Connection failed!");
    errorCount++;
    return false;
  }
  
  Serial.println("Connected successfully!");
  
  // Build the HTTP POST request manually
  // First line: POST /endpoint HTTP/1.1
  client.print("POST ");
  client.print(SERVER_ENDPOINT);
  client.println(" HTTP/1.1");
  
  // Headers
  client.print("Host: ");
  client.println(SERVER_ADDRESS);
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonPayload.length());
  client.println("Connection: close");
  client.println(); // End of headers
  
  // Request body (JSON payload)
  client.println(jsonPayload);
  
  // Log what we're sending
  Serial.println("---- HTTP Request ----");
  Serial.print("POST ");
  Serial.print(SERVER_ENDPOINT);
  Serial.println(" HTTP/1.1");
  Serial.print("Host: ");
  Serial.println(SERVER_ADDRESS);
  Serial.println("Content-Type: application/json");
  Serial.print("Content-Length: ");
  Serial.println(jsonPayload.length());
  Serial.println("Connection: close");
  Serial.println();
  Serial.println(jsonPayload);
  Serial.println("---------------------");
  
  // Wait for a response
  Serial.println("Waiting for response...");
  uint32_t startTime = millis();
  
  // Give the server time to respond (up to 5 seconds)
  while (client.connected() && !client.available() && millis() - startTime < 5000) {
    delay(100);
  }
  
  // Check if we received a response
  if (client.available()) {
    // Read and print the response line by line
    Serial.println("---- HTTP Response ----");
    String statusLine = client.readStringUntil('\n');
    Serial.println(statusLine);
    
    // Parse status code
    int statusCode = -1;
    if (statusLine.startsWith("HTTP/1.")) {
      statusCode = statusLine.substring(9, 12).toInt();
    }
    
    // Read headers
    bool headersEnd = false;
    while (client.available() && !headersEnd) {
      String line = client.readStringUntil('\n');
      line.trim();
      Serial.println(line);
      
      if (line.length() == 0) {
        headersEnd = true; // Empty line indicates end of headers
      }
    }
    
    // Read body (up to 1KB)
    if (client.available()) {
      char buffer[1024];
      int bytesRead = 0;
      
      Serial.println("---- Response Body ----");
      while (client.available() && bytesRead < sizeof(buffer) - 1) {
        buffer[bytesRead++] = client.read();
      }
      buffer[bytesRead] = '\0';
      Serial.println(buffer);
      Serial.println("----------------------");
    }
    
    // Check status code
    if (statusCode >= 200 && statusCode < 300) {
      Serial.println("Request successful with status code: " + String(statusCode));
      successCount++;
    } else {
      Serial.println("Request failed with status code: " + String(statusCode));
      errorCount++;
    }
  } else {
    Serial.println("No response received or connection closed!");
    errorCount++;
  }
  
  // Close the connection
  client.stop();
  Serial.println("Connection closed");
  
  return true;
}