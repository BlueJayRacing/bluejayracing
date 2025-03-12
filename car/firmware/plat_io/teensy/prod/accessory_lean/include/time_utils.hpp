#include <Arduino.h>
#include <TimeLib.h>

// Function to set up the correct time for the Teensy
void setupTime() {
    // Check if the current time is valid
    time_t t = now();
    
    // Set a default time if no time is set or if it's clearly invalid (2024-01-01 00:00:00)
    if (year(t) < 2023) {
        // Set a default time
        setTime(0, 0, 0, 1, 1, 2024);
        
        // Also set the internal RTC to this time
        Teensy3Clock.set(now());
        
        Serial.println("Setting default time to RTC: 2024-01-01 00:00:00");
    }
    
    // Print current time
    char timeStr[32];
    sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d", 
            year(), month(), day(), hour(), minute(), second());
    Serial.print("Current time: ");
    Serial.println(timeStr);
    
    // Ensure time is synced from RTC
    setSyncProvider(Teensy3Clock.get);
    setSyncInterval(300); // Sync every 5 minutes
    
    // Check if RTC was set
    if (timeStatus() != timeSet) {
        Serial.println("RTC sync failed!");
    } else {
        Serial.println("RTC sync successful");
    }
}