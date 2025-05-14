#include "led/led_functions.hpp"
#include "config/config.hpp"
#include <OctoWS2811.h>

namespace baja::led {

// ───────────────────────── Configuration ───────────────────────
// Single status LED configuration
const int NUM_PINS = 1;
byte pinList[NUM_PINS] = { baja::config::LED_STATUS_PIN };
const int LEDS_PER_STRIP = 1;
const int BYTES_PER_LED = 4;   // RGBW format
const int CONFIG = WS2811_GRBW | WS2811_800kHz;

// Memory allocation for OctoWS2811
DMAMEM int displayMem[LEDS_PER_STRIP * NUM_PINS * BYTES_PER_LED / 4];
int drawMem[LEDS_PER_STRIP * NUM_PINS * BYTES_PER_LED / 4];

// LED controller
OctoWS2811 leds(LEDS_PER_STRIP, displayMem, drawMem, CONFIG, NUM_PINS, pinList);

// ───────────────────────── State variables ───────────────────────
static bool inBootMode = true;
static SystemState* systemStatePtr = nullptr;  // Reference to external state
static SystemState lastDisplayedState = SystemState::READY;  // Track last displayed state

// ───────────────────────── Color helper function ───────────────────────
inline uint32_t makeColor(uint8_t r, uint8_t g, uint8_t b) {
    // Scale RGB values by the configured brightness
    uint8_t br = baja::config::LED_STATUS_BRIGHTNESS;
    r = (r * br) / 255;
    g = (g * br) / 255;
    b = (b * br) / 255;
    
    // Return in OctoWS2811 format (RGBW with W=0)
    return (r << 16) | (g << 8) | b;
}

// ───────────────────────── Public API ─────────────────────────────
void init(SystemState& stateRef) {
    // Store references to external state variables
    systemStatePtr = &stateRef;
    
    // Initialize the LED controller
    leds.begin();
    leds.show();  // Clear all LEDs
}

void startBoot() {
    inBootMode = true;
    
    // Set the status LED to BLUE during boot
    leds.setPixel(0, makeColor(0, 0, 255));
    leds.show();  // Display immediately

    lastDisplayedState = SystemState::BOOT;  // Reset last displayed state
}

void endBoot() {
    inBootMode = false;

    if (systemStatePtr) {
        *systemStatePtr = SystemState::READY;
    }
    
    // Force an update with the current state
    updateLED();
}

// ───────────────────────── Helper functions ─────────────────────────
void updateLED() {
    // Exit if we don't have valid state references
    if (!systemStatePtr) {
        return;
    }
    
    // Only update if the state has changed
    if (*systemStatePtr == lastDisplayedState && !inBootMode) {
        return;
    }
    
    // Select color based on the system state
    uint32_t color;
    
    switch (*systemStatePtr) {
        case SystemState::DATA_BAD:
            // Red - Critical data error (highest priority)
            color = makeColor(255, 0, 0);
            break;
            
        case SystemState::NO_NETWORK:
            // Orange - No network hardware
            color = makeColor(255, 30, 0);
            break;
            
        case SystemState::NO_CONNECTION:
            // Yellow - Network hardware present but no connection
            color = makeColor(255, 255, 0);
            break;
            
        case SystemState::NO_TIME:
            // Purple - Time not set correctly
            color = makeColor(128, 0, 128);
            break;
            
        case SystemState::READY:
        default:
            // Green - Everything OK
            color = makeColor(0, 255, 0);
            break;
    }
    
    // Update the LED
    leds.setPixel(0, color);
    leds.show();
    
    // Update the last displayed state
    lastDisplayedState = *systemStatePtr;
    
    // Boot mode is now definitely over
    inBootMode = false;
}

// ───────────────────────── Periodic update ─────────────────────────
void update() {
    // If in boot mode, don't change the LED state
    if (inBootMode) {
        return;
    }
    
    // Otherwise, update based on the current system state
    updateLED();
}

} // namespace baja::led