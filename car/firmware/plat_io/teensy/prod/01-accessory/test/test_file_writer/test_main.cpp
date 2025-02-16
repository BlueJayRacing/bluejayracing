#include <Arduino.h>
#include <unity.h>
#include <fileWriter.hpp>
#include <SD.h>

#include "verify_file_writer.hpp"


void setUp(void) {
}

void tearDown(void) {
}

void RUN_UNITY_TESTS() {
    UNITY_BEGIN();
    RUN_TEST(test_file_writer_error_handling);
    UNITY_END();
}

void setup() {
    SD.begin(BUILTIN_SDCARD);

    RUN_UNITY_TESTS();
}

void loop() {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(500);
}