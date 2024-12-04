#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>

static const char* TAG = "test";

void Test::testMemoryQueue(void)
{
    testMemoryQueueBasic();
    testMemoryQueueAcquireFull();
}

void Test::testMemoryQueueBasic(void)
{
    ESP_LOGI(TAG, "Starting to Test Basic Memory Queue Functions");
    memoryQueue queue(10, 20);
    ESP_LOGD(TAG, "Initialized memory queue");

    // Try to correctly acquire a block to write on
    memoryBlock* acquired_block = queue.acquire();
    assert(acquired_block != nullptr);
    ESP_LOGD(TAG, "successfully acquired block");

    std::vector<uint8_t> data_vec;
    data_vec.reserve(20);
    for (int i = 0; i < 20; i++) {
        data_vec.push_back(i);
    }

    assert(acquired_block->write(data_vec) == 20);
    assert(acquired_block->write(data_vec) == 0);
    ESP_LOGD(TAG, "successfully written to block");

    // Try to acquire a block to write on while acquire_lock is on
    assert(queue.acquire() == nullptr);
    ESP_LOGD(TAG, "successfully prevented acquiring two blocks at once");

    // Try to correctly push a block that you have finished writing to
    assert(queue.push(acquired_block) == 0);
    assert(queue.getNumPushed() == 1);
    ESP_LOGD(TAG, "successfully pushed acquired block");

    // Try to correctly pop a block to read from
    memoryBlock popped_block(20);
    assert(queue.pop(popped_block) == 0);
    ESP_LOGD(TAG, "successfully popped block from queue");

    uint8_t* data = popped_block.data();
    for (int i = 0; i < 20; i++) {
        assert(*(data + i) == i);
    }
    ESP_LOGI(TAG, "Passed Testing Basic Memory Queue Functions");
}

void Test::testMemoryQueueAcquireFull(void)
{
    ESP_LOGI(TAG, "Starting to Test Memory Queue When Full");
    memoryQueue queue(10, 20);

    for (int i = 0; i < 20; i++) {
        std::vector<uint8_t> data_vec(20, i);

        memoryBlock* acquired_block = queue.acquire();
        assert(acquired_block != nullptr);
        assert(acquired_block->write(data_vec) == 20);
        queue.push(acquired_block);
        ESP_LOGD(TAG, "Pushed Block %d", i);
    }

    memoryBlock mem_block(20);
    uint8_t* data = mem_block.data();

    for (int i = 0; i < queue.size(); i++) {
        assert(queue.pop(mem_block) == 0);
        for (int j = 0; j < mem_block.size(); j++) {
            ESP_LOGD(TAG, "Data val: %d, Index: %d", *(data + j), i);
            assert(*(data + j) == (i + 10));
        }
    }
    ESP_LOGI(TAG, "Passed Testing Memory Queue When Full");
}
