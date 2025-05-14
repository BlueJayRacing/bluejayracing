#ifndef defines_h
#define defines_h

#if !( defined(CORE_TEENSY) && defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41) )
  #error Only Teensy 4.1 supported
#endif

// Debug Level from 0 to 4
#define _AWS_TEENSY41_LOGLEVEL_             1
#define _ASYNC_UDP_TEENSY41_LOGLEVEL_       1

#define SHIELD_TYPE     "Teensy4.1 QNEthernet"

// #define USING_DHCP            true
#define USING_DHCP            false

#if !USING_DHCP
  // Set the static IP address to use if the DHCP fails to assign
  // IPAddress myIP(192, 168, 20, 222);
  // IPAddress myNetmask(255, 255, 255, 0);
  // IPAddress myGW(192, 168, 2, 1);
  // //IPAddress mydnsServer(192, 168, 2, 1);
  // IPAddress mydnsServer(8, 8, 8, 8);
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
// UDP Server settings (matches those in main.cpp)
// #define UDP_SERVER_ADDRESS "192.168.20.3"
// #define UDP_SERVER_PORT 8888

// Maximum UDP payload size (typical Ethernet MTU minus headers)
#define UDP_MAX_PAYLOAD_SIZE 1472

// HTTP Configuration (kept for compatibility)
#define HTTP_SERVER_ADDRESS "192.168.20.3"
#define HTTP_SERVER_PORT 9365
#define HTTP_SERVER_ENDPOINT "/"
#define HTTP_REQUEST_INTERVAL_MS 500   // 5 seconds interval between requests
#define DEFAULT_RX_TIMEOUT 15           // 15 seconds timeout for HTTP requests
#define HTTP_MAX_RETRIES 5              // Maximum number of reconnection attempts

// Error reporting
#define ERROR_REPORT_INTERVAL_MS 10000  // Only report errors every 10 seconds

#endif    //defines_h