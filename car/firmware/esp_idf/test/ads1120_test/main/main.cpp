#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>

#include <test.hpp>
#include <ads1120.hpp>

static const char* TAG = "main";

// Tested on a WSG-V2.0 board as labeled on board

extern "C" void app_main(void)
{
    Test test(ESP_LOG_DEBUG);

    test.testADS1120();
}