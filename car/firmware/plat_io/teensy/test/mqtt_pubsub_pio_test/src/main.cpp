/****************************************************************************************************************************
  AsyncDweetPOST.ino - Dead simple AsyncHTTPRequest for Teensy41 QNEthernet

  For Teensy41 with QNEthernet

  AsyncHTTPRequest_Teensy41 is a library for Teensy41 with QNEthernet

  Based on and modified from

  1. asyncHTTPrequest Library         (https://github.com/boblemaire/asyncHTTPrequest)
  2. AsyncHTTPRequest_Generic Library (https://github.com/khoih-prog/AsyncHTTPRequest_Generic)

  Built by Khoi Hoang https://github.com/khoih-prog/AsyncHTTPRequest_Teensy41
 *****************************************************************************************************************************/

// Dweet.io POST client. Connects to dweet.io once every ten seconds, sends a POST request and a request body.
// Shows how to use Strings to assemble path and body

#include "defines.h"

// Select a test server address
const char POST_ServerAddress[] = "http://192.168.137.84:9365/";

// use your own thing name here
String dweetName = "";

#define ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN_TARGET      "AsyncHTTPRequest_Teensy41 v1.10.0"
#define ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN             1010000

// Uncomment for certain HTTP site to optimize
//#define NOT_SEND_HEADER_AFTER_CONNECTED        true

// Level from 0-4
#define ASYNC_HTTP_DEBUG_PORT     Serial
#define _ASYNC_HTTP_LOGLEVEL_     10

// 600s = 10 minutes to not flooding, 60s in testing
#define HTTP_REQUEST_INTERVAL_MS     6000  //600000

// Seconds for timeout, default is 3s
#define DEFAULT_RX_TIMEOUT           10

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <AsyncHTTPRequest_Teensy41.h>          // https://github.com/khoih-prog/AsyncHTTPRequest_Teensy41

#include <Ticker.h>                             // https://github.com/sstaub/Ticker

AsyncHTTPRequest request;

void sendRequest();

// Repeat forever, millis() resolution
Ticker sendHTTPRequest(sendRequest, HTTP_REQUEST_INTERVAL_MS, 0, MILLIS);

void sendRequest()
{
  static bool requestOpenResult;

  if (request.readyState() == readyStateUnsent || request.readyState() == readyStateDone)
  {
    String postData = "sensorValue=";
    postData += analogRead(A0);

    Serial.println("\nMaking new POST request");

    requestOpenResult = request.open("POST", (POST_ServerAddress + dweetName + postData).c_str() );

    if (requestOpenResult)
    {
      // Only send() if open() returns true, or crash
      request.send();
    }
    else
    {
      Serial.println("Can't send bad request");
    }
  }
  else
  {
    Serial.println("Can't send request");
  }
}

void parseResponse(String responseText)
{
  /*
    Typical response is:
    {"this":"succeeded",
    "by":"getting",
    "the":"dweets",
    "with":[{"thing":"my-thing-name",
      "created":"2016-02-16T05:10:36.589Z",
      "content":{"sensorValue":456}}]}

    You want "content": numberValue
  */
  // now parse the response looking for "content":
  int labelStart = responseText.indexOf("content\":");
  // find the first { after "content":
  int contentStart = responseText.indexOf("{", labelStart);
  // find the following } and get what's between the braces:
  int contentEnd = responseText.indexOf("}", labelStart);
  String content = responseText.substring(contentStart + 1, contentEnd);

  Serial.println(content);

  // now get the value after the colon, and convert to an int:
  int valueStart = content.indexOf(":");
  String valueString = content.substring(valueStart + 1);
  int number = valueString.toInt();

  Serial.print("Value string: ");
  Serial.println(valueString);
  Serial.print("Actual value: ");
  Serial.println(number);
}

void requestCB(void* optParm, AsyncHTTPRequest* request, int readyState)
{
  (void) optParm;

  if (readyState == readyStateDone)
  {
    AHTTP_LOGWARN(F("\n**************************************"));
    AHTTP_LOGWARN1(F("Response Code = "), request->responseHTTPString());

    if (request->responseHTTPcode() == 200)
    {
      String responseText = request->responseText();

      Serial.println("\n**************************************");
      Serial.println(responseText);
      Serial.println("**************************************");

      parseResponse(responseText);
    }

    request->setDebug(true);
  }
}

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  Serial.print("\nStart AsyncDweetPOST on ");
  Serial.println(BOARD_NAME);
  Serial.println(ASYNC_HTTP_REQUEST_TEENSY41_VERSION);

#if defined(ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN)

  if (ASYNC_HTTP_REQUEST_TEENSY41_VERSION_INT < ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN)
  {
    Serial.print("Warning. Must use this example on Version equal or later than : ");
    Serial.println(ASYNC_HTTP_REQUEST_TEENSY41_VERSION_MIN_TARGET);
  }

#endif

  delay(500);

#if USING_DHCP
  // Start the Ethernet connection, using DHCP
  Serial.print("Initialize Ethernet using DHCP => ");
  Ethernet.begin();
#else
  // Start the Ethernet connection, using static IP
  Serial.print("Initialize Ethernet using static IP => ");
  Ethernet.begin(myIP, myNetmask, myGW);
  Ethernet.setDNSServerIP(mydnsServer);
#endif

  if (!Ethernet.waitForLocalIP(5000))
  {
    Serial.println(F("Failed to configure Ethernet"));

    if (!Ethernet.linkStatus())
    {
      Serial.println(F("Ethernet cable is not connected."));
    }

    // Stay here forever
    while (true)
    {
      delay(1);
    }
  }
  else
  {
    Serial.print(F("Connected! IP address:"));
    Serial.println(Ethernet.localIP());
  }

#if USING_DHCP
  delay(1000);
#else
  delay(2000);
#endif

  request.setDebug(true);

  request.onReadyStateChange(requestCB);
  sendHTTPRequest.start(); //start the ticker.
}

void loop()
{
  sendHTTPRequest.update();
}