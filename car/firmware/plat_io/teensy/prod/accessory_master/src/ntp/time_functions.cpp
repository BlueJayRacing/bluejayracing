// src/ntp/time_functions.cpp
#include "ntp/time_functions.hpp"
#include "config/defines.h"       // for configuration and thresholds
#include "util/debug_util.hpp"    // for logging (baja::util::Debug)
#include <stdlib.h>

// --- Internal constants and variables ---
static const int NTP_PACKET_SIZE = 48;
static uint8_t ntpPacketBuffer[NTP_PACKET_SIZE];

// Global offset tracking (in microseconds) for the fast RTC and SRTC.
static int32_t rtcOffsetUs = 0;
static int32_t srtcOffsetUs = 0;

// Threshold definitions (in µs)
#define RTC_UPDATE_THRESHOLD_US   2000     // Fast RTC update threshold: 500 µs
#define SRTC_UPDATE_THRESHOLD_US 50000     // SRTC update threshold: 50 ms

// List of NTP server IPs to try.
static IPAddress ntpServers[] = {
    IPAddress(208, 81, 1, 244),  // 0.ca.pool.ntp.org
    IPAddress(132, 163, 96, 1),   // time.nist.gov
    IPAddress(129, 6, 15, 28)     // Additional server.
};
static const size_t numNtpServers = sizeof(ntpServers) / sizeof(ntpServers[0]);
static size_t currentServerIndex = 0;

// Update interval (in milliseconds).
static const uint32_t UPDATE_INTERVAL_MS = 60000; // e.g. 60 sec.
static uint32_t lastNTPRequestTime = 0;

// Last successful NTP time (Unix epoch in seconds, UTC).
static time_t lastSuccessfulNTPTime = 0;

// --- AsyncUDP instance for NTP operations ---
static AsyncUDP ntpUdp;

// --- Helper functions for register access ---
// Read the high-precision RTC (HPRTC) as microseconds.
static uint64_t getRTCUs() {
    uint32_t hi = SNVS_HPRTCMR;
    uint32_t lo = SNVS_HPRTCLR;
    uint32_t secs = (hi << 17) | (lo >> 15);
    uint32_t frac = lo & 0x7FFF;
    return ((uint64_t)secs * 1000000ULL) + (((uint64_t)frac * 1000000ULL) / 32768);
}

// Read the battery-backed SRTC as microseconds.
static uint64_t getSRTCUs() {
    uint32_t srtcLow  = SNVS_LPSRTCLR;
    uint32_t srtcHigh = SNVS_LPSRTCMR;
    uint32_t secs = ((uint64_t)srtcHigh << 17) | (srtcLow >> 15);
    uint32_t fracTicks = srtcLow & 0x7FFF;
    uint32_t fracUs = ((uint64_t)fracTicks * 1000000ULL) / 32768;
    return ((uint64_t)secs * 1000000ULL) + fracUs;
}

// --- Function to create an NTP request packet ---
static void createNTPPacket() {
    memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);
    ntpPacketBuffer[0] = 0b11100011;   // LI, Version, Mode
    ntpPacketBuffer[1] = 0;            // Stratum
    ntpPacketBuffer[2] = 6;            // Polling Interval
    ntpPacketBuffer[3] = 0xEC;         // Precision
    // 8 bytes for Root Delay & Dispersion:
    ntpPacketBuffer[12] = 49;
    ntpPacketBuffer[13] = 0x4E;
    ntpPacketBuffer[14] = 49;
    ntpPacketBuffer[15] = 52;
}

// --- Function to send an NTP request ---
static void sendNTPRequest() {
    if (!ntpUdp.connected()) {
        if (!ntpUdp.connect(ntpServers[currentServerIndex], 123)) {
            baja::util::Debug::warning(F("Time: Failed to connect to NTP server"));
            return;
        }
    }
    createNTPPacket();
    ntpUdp.write(ntpPacketBuffer, NTP_PACKET_SIZE);
    baja::util::Debug::info(F("Time: NTP request sent to ") + String(ntpServers[currentServerIndex]));
}

// --- SRTC update function ---
// Stop both RTC and SRTC concurrently, write the new SRTC value, then restart them.
static void setSRTC(time_t t, uint16_t ms) {

    SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
    SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
    while ((SNVS_HPCR & SNVS_HPCR_RTC_EN) || (SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) { }

    SNVS_LPSRTCLR = (t << 15) + ((32768UL * ms) / 1000);
    SNVS_LPSRTCMR = t >> 17;

    SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
    while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) { }

    SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;

}

// --- HPRTC update function ---
// This sets the high-precision RTC registers (SNVS_HPRTCMR and SNVS_HPRTCLR) using the given time.
static void setHPRTC(uint64_t targetUs) {
  // Convert the target microseconds to whole seconds and remainder.
  uint32_t targetSeconds = targetUs / 1000000ULL;
  uint32_t remainderUs   = targetUs % 1000000ULL;
  // Convert the remainder (microseconds) to ticks at 32768 Hz.
  // One tick = 1/32768 sec, so fractional ticks = remainderUs * 32768 / 1000000.
  uint32_t fractionalTicks = (uint32_t)(((uint64_t)remainderUs * 32768ULL) / 1000000ULL);

  baja::util::Debug::info(F("Time: Setting HPRTC to ")
                          + String(targetSeconds) + F(" s, ")
                          + String(remainderUs) + F(" us"));

  // Stop the RTC by clearing the enable bits.
  SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
  while (SNVS_HPCR & SNVS_HPCR_RTC_EN) {
      // Wait until the RTC stops.
  }
  
  // Set the registers:
  // SNVS_HPRTCMR holds the high-order seconds bits.
  SNVS_HPRTCMR = targetSeconds >> 17;
  // SNVS_HPRTCLR holds the low-order seconds (in its upper 17 bits)
  // and the fractional ticks (in its lower 15 bits).
  SNVS_HPRTCLR = ((targetSeconds & ((1UL << 17) - 1)) << 15) | (fractionalTicks & 0x7FFF);
  
  // Restart the RTC.
  // SNVS_HPCR |= (SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
  SNVS_HPCR |= (SNVS_HPCR_RTC_EN);
}


// --- UDP packet callback for NTP responses ---
// Implements hierarchical updates: the HPRTC is updated if its diff > 500 µs,
// and the SRTC is updated if its diff > 50 ms. Also, we compensate the NTP time
// by adding 1 ms to account for UDP processing delay.
static void ntpPacketCallback(AsyncUDPPacket packet) {
    uint32_t t_start = micros();

    if (packet.length() < NTP_PACKET_SIZE) {
        return;
    }

    memcpy(ntpPacketBuffer, packet.data(), NTP_PACKET_SIZE);

    // --- Parse transmit timestamp (bytes 40-47) ---
    uint32_t secsSince1900 = ((uint32_t)ntpPacketBuffer[40] << 24) |
                             ((uint32_t)ntpPacketBuffer[41] << 16) |
                             ((uint32_t)ntpPacketBuffer[42] << 8)  |
                             ((uint32_t)ntpPacketBuffer[43]);
    uint32_t frac = ((uint32_t)ntpPacketBuffer[44] << 24) |
                    ((uint32_t)ntpPacketBuffer[45] << 16) |
                    ((uint32_t)ntpPacketBuffer[46] << 8)  |
                    ((uint32_t)ntpPacketBuffer[47]);
    const uint32_t seventyYears = 2208988800UL;
    time_t ntpTime = secsSince1900 - seventyYears;

    // Convert fractional part to microseconds.
    uint32_t microsecondsFraction = ((uint64_t)frac * 1000000ULL) >> 32;

    // --- Compensate for UDP delay: add 1ms (1000µs) ---
    uint64_t ntpFullUs = ((uint64_t)ntpTime * 1000000ULL) + microsecondsFraction;// + 1000ULL;

    // Recalculate seconds and ms fraction after compensation.
    time_t targetTime = ntpFullUs / 1000000ULL;
    uint16_t targetMs = (uint16_t)(((ntpFullUs % 1000000ULL) * 1000ULL + 500000ULL) / 1000000ULL);

    // Read current HPRTC in microseconds.
    uint64_t currentRTCUs = getRTCUs();
    int64_t diffRTC = (int64_t)ntpFullUs - (int64_t)currentRTCUs;
    baja::util::Debug::info(F("Time: Current HPRTC: ") + String(currentRTCUs) +
                            F(" us; Target: ") + String(ntpFullUs) +
                            F(" us; Diff: ") + String(diffRTC) + F(" us"));

    if (abs(diffRTC) > RTC_UPDATE_THRESHOLD_US) {
        // Update HPRTC using full microsecond resolution.
        setHPRTC(ntpFullUs);
        rtcOffsetUs = 0;
    } else {
        rtcOffsetUs = diffRTC;
        baja::util::Debug::info(F("Time: HPRTC diff within threshold; offset stored: ") +
                                String(rtcOffsetUs) + F(" us"));
    }


    // --- Hierarchical SRTC update ---
    uint64_t currentSRTCUs = getSRTCUs();
    int64_t diffSRTC = (int64_t)ntpFullUs - (int64_t)currentSRTCUs;
    if (abs(diffSRTC) > SRTC_UPDATE_THRESHOLD_US) {
        setSRTC(targetTime, targetMs);
        srtcOffsetUs = 0;
    } else {
        srtcOffsetUs = diffSRTC;
    }
    // --- Update Arduino TimeLib (for millis(), micros(), etc.) ---
    // This sets the Arduino runtime clock to targetTime.
    setTime(targetTime);
    lastSuccessfulNTPTime = targetTime;

    uint32_t totalElapsed = micros() - t_start;
    Serial.println("Total NTP callback: " + String(totalElapsed) + " us");
}

// --- Public API Functions ---
namespace baja {
namespace time {
namespace functions {

bool initialize() {
    lastNTPRequestTime = millis();
    lastSuccessfulNTPTime = 0;
    ntpUdp.onPacket(ntpPacketCallback);
    if (!ntpUdp.connect(ntpServers[currentServerIndex], 123)) {
        baja::util::Debug::warning(F("Time: Failed to connect to NTP server on initialization"));
    } else {
        baja::util::Debug::info(F("Time: UDP NTP client initialized"));
    }
    return true;
}

void update() {
    uint32_t start = micros();
    uint32_t nowMs = millis();
    if (nowMs - lastNTPRequestTime >= UPDATE_INTERVAL_MS) {
        sendNTPRequest();
        lastNTPRequestTime = nowMs;
        currentServerIndex = (currentServerIndex + 1) % numNtpServers;
    }
}

time_t getLastNTPTime() {
    return lastSuccessfulNTPTime;
}

} // namespace functions
} // namespace time
} // namespace baja
