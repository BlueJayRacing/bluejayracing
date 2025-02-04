#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <test.hpp>
#include <cstring>

static const char* TAG = "main";

extern "C" void app_main(void)
{
    Test test(ESP_LOG_DEBUG);

    test.testW25N04KV();
}