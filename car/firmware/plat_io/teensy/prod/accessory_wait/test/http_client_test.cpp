#include <Arduino.h>
#include <QNEthernet.h>
#include "defines.h"
#include <AsyncHTTPRequest_Teensy41.h>
#include <Ticker.h>

using namespace qindesign::network;

// Server URL - exact URL to your server
const char SERVER_URL[] = "http://192.168.137.84:9365/";

// HTTP request interval
#define HTTP_REQUEST_INTERVAL 3000  // 3 seconds

AsyncHTTPRequest request;

// Forward declaration
void sendRequest();

// Ticker to trigger HTTP requests periodically
Ticker httpTicker(sendRequest, HTTP_REQUEST_INTERVAL, 0, MILLIS);

// Callback for HTTP response
void requestCB(void* optParm, AsyncHTTPRequest* request, int readyState) {
  if (readyState == readyStateDone) {
    Serial.println("\n**************************************");
    Serial.print("Response Code = ");
    Serial.println(request->responseHTTPString());

    if (request->responseHTTPcode() == 200) {
      String responseText = request->responseText();
      Serial.println("Response:");
      Serial.println(responseText);
    }
    Serial.println("**************************************");
  }
}

// Send HTTP request - this closely follows the working example
void sendRequest() {
  if (request.readyState() == readyStateUnsent || request.readyState() == readyStateDone) {
    // Create simple data
    String postData = "{\"device\":\"Teensy41\",";
    postData += "\"timestamp\":" + String(millis()) + ",";
    postData += "\"data\":" + String(analogRead(A0));
    postData += "}";
    
    Serial.println("\nMaking new POST request");
    Serial.println("Data: " + postData);

    // Open request exactly like the example
    bool requestOpenResult = request.open("POST", SERVER_URL);

    if (requestOpenResult) {
      // Send the data
      bool success = request.send(postData);
      if (!success) {
        Serial.println("Failed to send request");
      }
    } else {
      Serial.println("Can't send bad request");
    }
  } else {
    Serial.println("Can't send request - previous request still in progress");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  Serial.println("\nDirect HTTP POST Example");
  Serial.println("========================");
  
  // Initialize analog pin
  pinMode(A0, INPUT);

  // Initialize Ethernet
#if USING_DHCP
  Serial.print("Initializing Ethernet with DHCP... ");
  Ethernet.begin();
#else
  Serial.print("Initializing Ethernet with static IP... ");
  Ethernet.begin(myIP, myNetmask, myGW);
  Ethernet.setDNSServerIP(mydnsServer);
#endif

  if (!Ethernet.waitForLocalIP(5000)) {
    Serial.println("Failed to configure Ethernet");
    if (!Ethernet.linkStatus()) {
      Serial.println("Ethernet cable is not connected.");
    }
    while (true) delay(1000);
  } else {
    char ipStr[16];
    
    Serial.print("Connected! IP address: ");
    Serial.println(Ethernet.localIP());
    Serial.println(ipStr);
  }

  // Set up request
  request.setDebug(true);
  request.onReadyStateChange(requestCB);
  
  // Start the ticker
  httpTicker.start();
  
  // Send initial request
  delay(1000);
  sendRequest();
}

void loop() {
  httpTicker.update();
}