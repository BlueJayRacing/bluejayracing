/****************************************************************************************************************************
  AsyncUdpNTPSetSRTC_Microseconds.ino

  This example uses the AsyncUDP_Teensy41 library to fetch UTC time from an NTP server,
  prints the NTP time, the current SRTC (battery-backed) time, and the high-resolution RTC time (in microseconds),
  then updates the SRTC (and TimeLib) if there is a mismatch.
  
  The SRTC is updated by writing to SNVS_LPSRTCLR and SNVS_LPSRTCMR, then restarting the clocks.
  
  This ensures that regardless of the timezone used during programming, the system ends up with UTC.
*****************************************************************************************************************************/

#include "defines.h"        // from the AsyncUDP_Teensy41 example
#include <Ticker.h>         // For periodic NTP requests
#include <TimeLib.h>        // For time functions
#include "Arduino.h"

// NTP configuration
IPAddress timeServerIP = IPAddress(208, 81, 1, 244);  // Example: 0.ca.pool.ntp.org
#define NTP_REQUEST_PORT      123
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

// AsyncUDP instance
AsyncUDP Udp;

// Send an NTP request every 60 seconds (adjust as needed)
#define UDP_REQUEST_INTERVAL_MS     60000  

void sendNTPPacket();
Ticker sendUDPRequest(sendNTPPacket, UDP_REQUEST_INTERVAL_MS, 0, MILLIS);

//-----------------------------------------------------
// Create an NTP packet in packetBuffer
void createNTPpacket() {
  Serial.println("Creating NTP packet...");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum, or type of clock
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  // 8 bytes for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
}

//-----------------------------------------------------
// Read high-resolution RTC time (battery-backed RTC registers) in microseconds.
// The RTC registers (SNVS_HPRTCMR and SNVS_HPRTCLR) contain the time in ticks at 32.768 kHz.
uint64_t getMicrosecondsSinceEpoch() {
  uint32_t hi = SNVS_HPRTCMR;
  uint32_t lo = SNVS_HPRTCLR;
  // The RTC seconds are formed as: seconds = (hi << 17) | (lo >> 15)
  uint32_t secs = (hi << 17) | (lo >> 15);
  // The fractional part (ticks within the second) are the lower 15 bits.
  uint32_t frac = lo & 0x7FFF;
  // Convert to microseconds.
  return ((uint64_t)secs * 1000000ULL) + (((uint64_t)frac * 1000000ULL) / 32768);
}

//-----------------------------------------------------
// Read the SRTC (Secure RTC) time from the battery-backed registers.
// The SRTC was set by writing: SNVS_LPSRTCLR = t << 15; SNVS_LPSRTCMR = t >> 17;
// To reconstruct t (in seconds), we do:
uint64_t getSRTC() {
  uint32_t srtcLow  = SNVS_LPSRTCLR;
  uint32_t srtcHigh = SNVS_LPSRTCMR;
  uint64_t t = (((uint64_t)srtcHigh) << 17) | (srtcLow >> 15);
  return t; // in seconds (Unix epoch seconds, if set that way)
}

//-----------------------------------------------------
// Set the SRTC with a given Unix epoch time (seconds).
// This function stops the RTC and SRTC, writes the new time (converted to 32.768Hz ticks),
// then restarts and synchronizes the RTC from the SRTC.
void setSRTC(time_t t) {
  Serial.print("Setting SRTC to: ");
  Serial.println(t);
  
  // Stop the RTC (volatile clock)
  SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
  while (SNVS_HPCR & SNVS_HPCR_RTC_EN);
  
  // Stop the SRTC (battery-backed clock)
  SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
  while (SNVS_LPCR & SNVS_LPCR_SRTC_ENV);
  
  // Write the new time:
  // Convert seconds t to ticks at 32.768 kHz:
  SNVS_LPSRTCLR = t << 15;  // Lower 32 bits (t * 32768)
  SNVS_LPSRTCMR = t >> 17;   // Upper bits
       
  // Start the SRTC
  SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
  while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV));
  
  // Restart the RTC and synchronize it to the SRTC
  SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;
  
  Serial.println("SRTC update complete.");
}

//-----------------------------------------------------
// Standard sync provider for TimeLib using Teensy3's RTC.
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

//-----------------------------------------------------
// Parse the incoming NTP packet, log details, update the SRTC and TimeLib.
// This function also prints the microseconds since epoch and logs the SRTC state before and after.
void parsePacket(AsyncUDPPacket packet) {
  memcpy(packetBuffer, packet.data(), sizeof(packetBuffer));
  Serial.println("\n--- Received NTP Packet ---");
  Serial.print("From: ");
  Serial.print(packet.remoteIP());
  Serial.print(":");
  Serial.println(packet.remotePort());
  
  // Extract the NTP timestamp (transmit timestamp begins at byte 40)
  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord  = word(packetBuffer[42], packetBuffer[43]);
  unsigned long secsSince1900 = (highWord << 16) | lowWord;
  const unsigned long seventyYears = 2208988800UL;
  unsigned long epoch = secsSince1900 - seventyYears;
  time_t ntpTime = epoch;
  
  Serial.print("NTP seconds since Jan 1 1900: ");
  Serial.println(secsSince1900);
  Serial.print("Converted Unix epoch time (UTC): ");
  Serial.println(ntpTime);
  
  // Log the current SRTC reading and RTC high-resolution timestamp.
  uint64_t currentSRTC = getSRTC();
  uint64_t currentRTCUs = getMicrosecondsSinceEpoch();
  Serial.print("Current SRTC time (seconds): ");
  Serial.println(currentSRTC);
  Serial.print("Current RTC time (microseconds): ");
  Serial.println(currentRTCUs);
  
  // If the SRTC time is different from the NTP time, update it.
  if (ntpTime != (time_t)currentSRTC) {
    Serial.print("Updating SRTC from ");
    Serial.print(currentSRTC);
    Serial.print(" to ");
    Serial.println(ntpTime);
    setSRTC(ntpTime);
  } else {
    Serial.println("SRTC time is already correct.");
  }
  
  // Update TimeLib with the new UTC time.
  setTime(ntpTime);
  Serial.print("TimeLib updated to: ");
  Serial.println(ntpTime);
  
  // Log the updated RTC high-resolution timestamp.
  uint64_t updatedRTCUs = getMicrosecondsSinceEpoch();
  Serial.print("Updated RTC time (microseconds): ");
  Serial.println(updatedRTCUs);
}

//-----------------------------------------------------
// Send an NTP request packet.
void sendNTPPacket() {
  createNTPpacket();
  Udp.write(packetBuffer, sizeof(packetBuffer));
}

//-----------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);
  Serial.println("\nStarting AsyncUDP NTP SRTC Setter with Microsecond Logging Example");
  
  // Initialize Ethernet (using DHCP or static IP as defined in defines.h)
#if USING_DHCP
  Serial.print("Initializing Ethernet using DHCP => ");
  Ethernet.begin();
#else
  Serial.print("Initializing Ethernet using static IP => ");
  Ethernet.begin(myIP, myNetmask, myGW);
  Ethernet.setDNSServerIP(mydnsServer);
#endif

  if (!Ethernet.waitForLocalIP(5000)) {
    Serial.println(F("Failed to configure Ethernet"));
    if (!Ethernet.linkStatus()) {
      Serial.println(F("Ethernet cable is not connected."));
    }
    while (true) { delay(1); }
  } else {
    Serial.print("Connected! IP address: ");
    Serial.println(Ethernet.localIP());
  }
  
  delay(1000);
  
  // Connect UDP to the NTP server at port 123.
  if (Udp.connect(timeServerIP, NTP_REQUEST_PORT)) {
    Serial.println("UDP connected to NTP server");
    Udp.onPacket([](AsyncUDPPacket packet) {
      parsePacket(packet);
    });
  }
  
  // Start the ticker to send NTP requests periodically.
  sendUDPRequest.start();
  
  // Set the initial TimeLib sync provider.
  setSyncProvider(getTeensy3Time);
}

//-----------------------------------------------------
// Print current time and RTC microsecond timestamp.
void printTime() {
  Serial.print("Time: ");
  Serial.print(hour());
  Serial.print(":");
  if (minute() < 10) Serial.print("0");
  Serial.print(minute());
  Serial.print(":");
  if (second() < 10) Serial.print("0");
  Serial.print(second());
  Serial.print("  Date: ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());
  Serial.print("  RTC (µs): ");
  Serial.println(getMicrosecondsSinceEpoch());
}

void loop() {
  sendUDPRequest.update();
  printTime();
  delay(1000);
}
