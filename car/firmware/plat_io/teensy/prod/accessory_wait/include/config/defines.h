#ifndef defines_h
#define defines_h

#if !( defined(CORE_TEENSY) && defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41) )
  #error Only Teensy 4.1 supported
#endif

// Debug Level from 0 to 4
#define _TEENSY41_ASYNC_TCP_LOGLEVEL_       4
#define _AWS_TEENSY41_LOGLEVEL_             4

#define SHIELD_TYPE     "Teensy4.1 QNEthernet"

#if (_AWS_TEENSY41_LOGLEVEL_ > 3)
  #warning Using QNEthernet lib for Teensy 4.1. Must also use Teensy Packages Patch or error
#endif

#define USING_DHCP            true
//#define USING_DHCP            false

#if !USING_DHCP
  // Set the static IP address to use if the DHCP fails to assign
  IPAddress myIP(192, 168, 2, 222);
  IPAddress myNetmask(255, 255, 255, 0);
  IPAddress myGW(192, 168, 2, 1);
  //IPAddress mydnsServer(192, 168, 2, 1);
  IPAddress mydnsServer(8, 8, 8, 8);
#endif

#include "QNEthernet.h"       // https://github.com/ssilverman/QNEthernet
using namespace qindesign::network;

// Define QNEthernet LinkStatus values to fix compiler error
const int LinkStatus_kLinkStatusDown = 0;
const int LinkStatus_kLinkStatusUp = 1;

// QNEthernet settings for better reliability
#define QNETHERNET_MEMORY_POOL_SIZE 4096
#define QNETHERNET_MAX_TXNS 16

// Connection parameters
#define HTTP_SERVER_ADDRESS "192.168.20.3"
#define HTTP_SERVER_PORT 9365
#define HTTP_SERVER_ENDPOINT "/"

// HTTP Configuration
#define HTTP_REQUEST_INTERVAL_MS 200    // 200ms interval between requests (faster with persistent connections)
#define DEFAULT_RX_TIMEOUT 5            // 5 seconds timeout for HTTP requests
#define HTTP_MAX_RETRIES 5              // Maximum number of reconnection attempts
#define HTTP_MAX_IDLE_CONNECTION_TIME 30000  // Close persistent connection after 30 seconds of inactivity
#define HTTP_READ_TIMEOUT 3000          // Timeout for reading responses (3 seconds)
#define HTTP_CONNECT_TIMEOUT 5000       // Timeout for establishing connection (5 seconds)

// Use persistent connections by default (much more efficient)
#define HTTP_USE_PERSISTENT_CONNECTION true

// Adjust Ethernet socket behavior for persistent connections
#define HTTP_SOCKET_TIMEOUT 500         // TCP socket timeout in ms
#define HTTP_DEFER_LINGER_TIME 1        // Enable linger with 1 second timeout

// Error reporting
#define ERROR_REPORT_INTERVAL_MS 10000  // Only report errors every 10 seconds

#endif    //defines_h