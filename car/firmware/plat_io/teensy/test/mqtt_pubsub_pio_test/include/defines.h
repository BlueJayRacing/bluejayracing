/****************************************************************************************************************************
  defines.h

  Dead simple AsyncHTTPRequest for Teensy41 QNEthernet

  For Teensy41 with QNEthernet

  AsyncHTTPRequest_Teensy41 is a library for Teensy41 with QNEthernet

  Based on and modified from

  1. asyncHTTPrequest Library         (https://github.com/boblemaire/asyncHTTPrequest)
  2. AsyncHTTPRequest_Generic Library (https://github.com/khoih-prog/AsyncHTTPRequest_Generic)

  Built by Khoi Hoang https://github.com/khoih-prog/AsyncHTTPRequest_Teensy41
 *****************************************************************************************************************************/

 #ifndef defines_h
 #define defines_h
 
 #if !( defined(CORE_TEENSY) && defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41) )
   #error Only Teensy 4.1 supported
 #endif
 
 // Debug Level from 0 to 4
 #define _TEENSY41_ASYNC_TCP_LOGLEVEL_       1
 #define _AWS_TEENSY41_LOGLEVEL_             1
 
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
 
 //#include <AsyncHTTPRequest_Teensy41.h>
 
 #endif    //defines_h