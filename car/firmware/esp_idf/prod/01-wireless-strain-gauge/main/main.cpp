#include <assert.h>
#include <esp_log.h>
#include <stdio.h>

#include <memoryBlock.hpp>
#include <memoryQueue.hpp>

#include <ad5626.hpp>

#include <test.hpp>

static const char* TAG = "main";

extern "C" void app_main(void)
{
    memoryQueue queue(10, 20);
    int index = 0;

    while (true) {
        ESP_LOGI(TAG, "testing index %d", index);
        test_memory_queue_acquire_full(&queue);
        index = (index + 1) % 10;
        vTaskDelay(100);
    }
}