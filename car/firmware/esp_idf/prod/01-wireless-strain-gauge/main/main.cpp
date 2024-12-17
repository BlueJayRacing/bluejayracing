#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>

#include <memoryBlock.hpp>
#include <memoryQueue.hpp>

#include <ad5626.hpp>

#include <test.hpp>

static const char* TAG = "main";

extern "C" void app_main(void)
{
    Test test(ESP_LOG_DEBUG);
    test.testMQTTManager();
}