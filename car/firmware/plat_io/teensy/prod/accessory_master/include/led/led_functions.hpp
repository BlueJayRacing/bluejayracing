#pragma once
#include <Arduino.h>

// Public LED API for the whole project
namespace baja::led {

// Enhanced semantic state enum
enum class SystemState : uint8_t {
    READY,          // green - everything is working
    DATA_BAD,       // red - critical data error (highest priority)
    NO_NETWORK,     // orange - no network hardware
    NO_CONNECTION,  // yellow - network hardware present but no connection
    NO_TIME,        // purple - time not set correctly
    BOOT            // blue - boot mode (LED is blue during boot sequence)
};

// ───────────────────────── Initialisation ──────────────────────────
void init(SystemState& stateRef);
void startBoot();                        // Start boot sequence with blue LED
void endBoot();                          // End boot sequence and set to current state

// ───────────────────────── Periodic update (non‑blocking) ─────────────
void update();                          // Update LED based on current system state
void updateLED();                     // Update LED based on current system state (non‑blocking)

} // namespace baja::led