# Comprehensive Documentation for Teensy 4.1 Timekeeping and i.MX RT1062 Clocks

This document details the design and implementation of the timekeeping subsystem on the Teensy 4.1 (based on the i.MX RT1062) with a focus on clock management, register usage, timing conversions, update strategies, and atomic operations. It combines high‑level design rationale with code-level explanations to provide a complete view of how the high‐precision RTC (HPRTC) and secure, battery‑backed RTC (SRTC) work together without using the TS command to sync the RTC back to the SRTC.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Overview of Clock Domains](#overview-of-clock-domains)
3. [Register Details](#register-details)
    - [High-Precision RTC Registers](#high-precision-rtc-registers)
    - [Secure RTC Registers](#secure-rtc-registers)
    - [Control Registers](#control-registers)
4. [Timing Calculations and Conversions](#timing-calculations-and-conversions)
5. [Hierarchical Update Strategy](#hierarchical-update-strategy)
6. [Atomic Update Procedures](#atomic-update-procedures)
7. [Code Implementation Details](#code-implementation-details)
8. [Key Design Decisions](#key-design-decisions)
9. [References](#references)

---

## Introduction

The Teensy 4.1 board uses the NXP i.MX RT1062 microcontroller, which provides a robust timekeeping subsystem consisting of a fast, volatile high‑precision RTC (HPRTC) and a secure, battery‑backed RTC (SRTC). This documentation explains the underlying register configurations, timing calculations, and update protocols implemented in the provided code.

---

## Overview of Clock Domains

The i.MX RT1062 offers multiple clock domains. In our design:

- **High-Precision RTC (HPRTC):**  
  - Uses a 32.768 kHz reference clock.
  - Provides a resolution of approximately 30.5 µs per tick.
  - Designed for fast, runtime timing.
  
- **Secure RTC (SRTC):**  
  - Battery‑backed to preserve time across power cycles.
  - Uses a similar tick resolution for sub‑second timing.
  - Updated less frequently due to slower update routines.

Both clocks store time as a composite value—splitting the whole seconds and a fractional part (ticks).

---

## Register Details

### High-Precision RTC Registers

- **SNVS_HPRTCMR:**  
  Holds the high-order seconds bits. For time conversion, the full seconds value is obtained by combining this register with the lower portion from SNVS_HPRTCLR.

- **SNVS_HPRTCLR:**  
  Contains:
  - The lower 17 bits for the seconds (when shifted left by 15).
  - The lower 15 bits for the fractional (sub‑second) ticks (0–32767).

### Secure RTC Registers

- **SNVS_LPSRTCLR:**  
  Stores the lower portion of the seconds (shifted left by 15) along with the 15‑bit fractional ticks.

- **SNVS_LPSRTCMR:**  
  Holds the upper bits of the seconds value.

### Control Registers

- **SNVS_HPCR (High-Power Control Register):**  
  - **SNVS_HPCR_RTC_EN:** Enables the HPRTC.
  - **SNVS_HPCR_HP_TS:** Enables high‑precision timestamping.  
    _Note:_ In our code, when syncing the RTC, we deliberately do **not** re-enable the TS (timestamp) feature after updating the registers. This avoids using the TS command to sync the RTC back to the SRTC.

- **SNVS_LPCR (Low-Power Control Register):**  
  - **SNVS_LPCR_SRTC_ENV:** Enables the secure (battery‑backed) SRTC.

---

## Timing Calculations and Conversions

Time is represented in microseconds by combining whole seconds with the fractional tick value. The conversion formulas used are:

- **For HPRTC/SRTC:**  
  ```
  microseconds = (seconds × 1,000,000) + ((fractional ticks × 1,000,000) / 32768)
  ```

- **Setting the Time:**  
  A full microseconds value is split into:
  - **Whole seconds:** `targetSeconds = targetUs / 1000000`
  - **Remainder:** `remainderUs = targetUs % 1000000`
  - **Fractional ticks:**  
    ```
    fractionalTicks = (remainderUs × 32768) / 1000000
    ```

This conversion ensures both the seconds and sub‑second portions maintain full microsecond precision.

---

## Hierarchical Update Strategy

The update strategy uses a two‑tier approach based on the error between the current RTC values and the reference NTP time:

- **Fast RTC (HPRTC) Update:**  
  - If the difference between the NTP time and the HPRTC time exceeds a lower threshold (e.g., 500 µs as noted in design documentation—even though the code defines a threshold value, the intent is to perform a fast update for small discrepancies), the HPRTC registers are updated immediately.
  - If the difference is within the threshold, an offset (`rtcOffsetUs`) is stored and applied during runtime reads.

- **Secure RTC (SRTC) Update:**  
  - Updated less frequently because SRTC updates take longer (approximately 1 ms).
  - An update is performed only when the difference exceeds a higher threshold (e.g., 50 ms).
  - Otherwise, a secure offset (`srtcOffsetUs`) is maintained.

This dual-threshold mechanism minimizes unnecessary writes to the battery‑backed clock while keeping the runtime time as accurate as possible.

---

## Atomic Update Procedures

To ensure consistent time updates and prevent register rollover issues, both the HPRTC and SRTC are updated atomically by following this sequence:

1. **Disable Clocks:**  
   - Clear the enable bits (SNVS_HPCR_RTC_EN and SNVS_HPCR_HP_TS for HPRTC; SNVS_LPCR_SRTC_ENV for SRTC).
2. **Wait for Confirmation:**  
   - Loop until the registers confirm that the clocks have stopped.
3. **Write New Time Values:**  
   - Update the seconds and fractional ticks by writing to SNVS_HPRTCMR/SNVS_HPRTCLR (for HPRTC) and SNVS_LPSRTCLR/SNVS_LPSRTCMR (for SRTC).
4. **Re-enable Clocks:**  
   - Set the corresponding enable bits.
5. **Wait for Re-enable Confirmation:**  
   - Loop until the registers indicate the clocks are active again.

This process minimizes drift or inconsistencies due to simultaneous time reads or rollover events.

---

## Code Implementation Details

The following code excerpts illustrate key parts of the implementation:

### Reading Time from HPRTC

```cpp
uint64_t getMicrosecondsSinceEpoch_() {
  // Atomic read loop: read RTC registers until two consecutive reads are identical.
  uint32_t hi1 = SNVS_HPRTCMR;
  uint32_t lo1 = SNVS_HPRTCLR;
  while (true) {
    uint32_t hi2 = SNVS_HPRTCMR;
    uint32_t lo2 = SNVS_HPRTCLR;
    if (hi1 == hi2 && lo1 == lo2) {
      uint32_t secs = (hi2 << 17) | (lo2 >> 15);
      uint32_t frac = lo2 & 0x7FFF;
      uint64_t us = ((uint64_t)secs * 1000000ULL) + (((uint64_t)frac * 1000000ULL) / 32768);
      return us;
    }
    hi1 = hi2;
    lo1 = lo2;
  }
}
```

### Setting the High-Precision RTC

```cpp
static void setHPRTC(uint64_t targetUs) {
  uint32_t targetSeconds = targetUs / 1000000ULL;
  uint32_t remainderUs   = targetUs % 1000000ULL;
  uint32_t fractionalTicks = (uint32_t)(((uint64_t)remainderUs * 32768ULL) / 1000000ULL);

  // Log the target time for debugging
  baja::util::Debug::info(F("Time: Setting HPRTC to ") +
                          String(targetSeconds) + F(" s, ") +
                          String(remainderUs) + F(" us"));

  // Stop the RTC by clearing the enable bits.
  SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
  while (SNVS_HPCR & SNVS_HPCR_RTC_EN) { }

  // Set the registers:
  SNVS_HPRTCMR = targetSeconds >> 17;
  SNVS_HPRTCLR = ((targetSeconds & ((1UL << 17) - 1)) << 15) | (fractionalTicks & 0x7FFF);
  
  // Re-enable the RTC.
  // Note: Only the RTC enable bit (SNVS_HPCR_RTC_EN) is re-enabled,
  // intentionally leaving SNVS_HPCR_HP_TS disabled to avoid syncing via the TS command.
  SNVS_HPCR |= (SNVS_HPCR_RTC_EN);
}
```

### Updating the Secure RTC

```cpp
static void setSRTC(time_t t, uint16_t ms) {
  // Stop both RTC and SRTC concurrently
  SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
  SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
  while ((SNVS_HPCR & SNVS_HPCR_RTC_EN) || (SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) { }

  // Update SRTC registers with new time value
  SNVS_LPSRTCLR = (t << 15) + ((32768UL * ms) / 1000);
  SNVS_LPSRTCMR = t >> 17;

  // Restart SRTC and confirm re-enable
  SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
  while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)) { }

  // Restart HPRTC
  SNVS_HPCR |= (SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
}
```

### Hierarchical Update in the NTP Callback

In the NTP callback, the time difference is computed and compared against defined thresholds. Depending on the result:

- If the HPRTC error exceeds the threshold:
  - `setHPRTC()` is called to update the fast clock.
- If within the threshold:
  - An offset is stored to adjust runtime reads.

Similarly for SRTC:
- `setSRTC()` is invoked only when the difference exceeds a higher threshold.

---

## Key Design Decisions

### Separation of Clocks
The design leverages a fast, volatile RTC for runtime timing (HPRTC) and a slower, battery‑backed RTC for persistent timekeeping (SRTC). This separation allows for frequent small adjustments on the HPRTC without incurring the overhead of updating the SRTC unnecessarily.

### Atomic Updates
Disabling clocks during updates ensures that there are no intermediate states where one clock is updated and the other is not, preserving synchronization and minimizing drift.

### No TS Command for RTC Sync
In the update routine for the HPRTC, the high‑precision timestamp (TS) bit is intentionally left disabled when re‑enabling the clock. This design choice avoids using the TS command to sync the RTC back to the SRTC, ensuring that updates are performed only through direct register writes. This approach minimizes potential conflicts or side effects that might arise from automated timestamp syncing.

### Offset Tracking
For small time discrepancies, rather than performing a full register update (which may be time‑consuming and potentially introduce drift), the system tracks an offset. This allows for continuous correction during runtime without expensive register reprogramming.

---

## Conclusion

This document provides a detailed view of the Teensy 4.1 timekeeping subsystem on the i.MX RT1062. By carefully managing register updates, timing conversions, and hierarchical update strategies, the design achieves high‑precision timekeeping with both rapid runtime performance and robust persistence. Notably, the design purposefully avoids using the TS command for syncing—opting instead for direct, atomic updates—to minimize potential timing inconsistencies.

---

## References

- i.MX RT1060/1062 Processor Reference Manual and Datasheet (NXP)
- PJRC Teensy 4.1 Technical Information and Schematics
- Forum discussions on Teensy 4.1 clock design and i.MX RT1062 register usage
- MCUXpresso SDK documentation and configuration tools