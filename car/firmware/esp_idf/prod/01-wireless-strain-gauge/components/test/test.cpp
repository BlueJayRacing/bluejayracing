#include <esp_log.h>
#include <esp_system.h>

#include <test.hpp>

static const char* TAG = "test";

void test_memory_queue(memoryQueue* queue)
{
    // Try to correctly acquire a block to write on
    memoryBlock* acquired_block = queue->acquire();
    assert(acquired_block != nullptr);
    ESP_LOGI(TAG, "successfully acquired block");

    std::vector<uint8_t> data_vec;
    data_vec.reserve(20);
    for (int i = 0; i < 20; i++) {
        data_vec.push_back(i);
    }

    assert(acquired_block->write(data_vec) == 20);
    assert(acquired_block->write(data_vec) == 0);
    ESP_LOGI(TAG, "successfully written to block");

    // Try to acquire a block to write on while acquire_lock is on
    assert(queue->acquire() == nullptr);
    ESP_LOGI(TAG, "successfully prevented acquiring two blocks at once");

    // Try to correctly push a block that you have finished writing to
    assert(queue->push(acquired_block) == 0);
    assert(queue->getNumPushed() == 1);
    ESP_LOGI(TAG, "successfully pushed acquired block");

    // Try to correctly pop a block to read from
    memoryBlock popped_block(20);
    assert(queue->pop(popped_block) == 0);
    ESP_LOGI(TAG, "successfully popped block from queue");

    uint8_t* data = popped_block.data();
    for (int i = 0; i < 20; i++) {
        assert(*(data + i) == i);
    }
}

void test_memory_queue_acquire_full(memoryQueue* queue)
{
    for (int i = 0; i < 20; i++) {
        std::vector<uint8_t> data_vec(20, i);

        memoryBlock* acquired_block = queue->acquire();
        assert(acquired_block != nullptr);
        assert(acquired_block->write(data_vec) == 20);
        queue->push(acquired_block);
        ESP_LOGI(TAG, "Pushed Block %d", i);
    }

    memoryBlock mem_block(20);
    uint8_t* data = mem_block.data();

    for (int i = 0; i < queue->size(); i++) {
        assert(queue->pop(mem_block) == 0);
        for (int j = 0; j < mem_block.size(); j++) {
            ESP_LOGI(TAG, "Data val: %d, Index: %d", *(data + j), i);
            assert(*(data + j) == (i + 10));
        }
    }
}
