#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>

#include <test.hpp>

#include <w25n04kv.hpp>

static const char* TAG = "main";

extern "C" void app_main(void)
{
    Test test(ESP_LOG_VERBOSE);
    ESP_LOGI(TAG, "Starting tests");
    test.testSPIFlash();
}