#include <Arduino.h>
#include <unity.h>

#include "verify_file_writer.hpp"

void RUN_UNITY_TESTS() {
    UNITY_BEGIN();
    UNITY_END();
}

void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    RUN_UNITY_TESTS();
}

void loop() {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}