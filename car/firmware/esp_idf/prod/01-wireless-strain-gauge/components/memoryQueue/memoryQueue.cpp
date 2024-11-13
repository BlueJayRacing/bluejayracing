#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <string.h>

#include <memoryQueue.hpp>

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

/*******************************************************************************
 * @brief Acquire a memory block from the queue to write data to.
 *
 * @return Returns a pointer to the memory block if successful, or nullptr on
 *         failure.
 *
 * @note You can only acquire one memory block at a time.
 *******************************************************************************/
memoryBlock* memoryQueue::acquire(void)
{
    taskENTER_CRITICAL(&my_spinlock);
    if (acquired_lock_) {
        return nullptr;
    }

    if (num_pushed_ == block_vec_.size()) {
        block_vec_.at(back_index_).clear();
        num_pushed_--;
        front_index_ = (front_index_ + 1) % block_vec_.size();
    }

    acquired_lock_ = true;
    taskEXIT_CRITICAL(&my_spinlock);

    return &(block_vec_.at(back_index_));
}

/*******************************************************************************
 * @brief Push a memory block that's been used back to the queue.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int8_t memoryQueue::push(memoryBlock* t_mem_block)
{
    taskENTER_CRITICAL(&my_spinlock);
    if (t_mem_block != &(block_vec_.at(back_index_))) {
        return -1;
    }

    acquired_lock_ = false;
    back_index_    = (back_index_ + 1) % block_vec_.size();
    num_pushed_++;
    taskEXIT_CRITICAL(&my_spinlock);

    return 0;
}

/*******************************************************************************
 * @brief Pop a memory block on the queue by copying all data.
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
int8_t memoryQueue::pop(memoryBlock& copy_block)
{
    taskENTER_CRITICAL(&my_spinlock);
    if (num_pushed_ == 0 || copy_block.size() != block_vec_.at(0).size()) {
        return -1;
    }

    memcpy(copy_block.data(), block_vec_.at(front_index_).data(), copy_block.size());
    block_vec_.at(front_index_).clear();
    front_index_ = (front_index_ + 1) % block_vec_.size();
    num_pushed_--;
    taskEXIT_CRITICAL(&my_spinlock);

    return 0;
}

/*******************************************************************************
 * @brief Get the number of pushed blocks that's currently on the queue.
 *
 * @return Returns the current number of pushed blocks.
 *******************************************************************************/
uint16_t memoryQueue::getNumPushed(void) { return num_pushed_; }

/*******************************************************************************
 * @brief Get the size of the memory Queue (i.e. get the total number of mem blocks).
 *
 * @return Returns the total number of memory blocks.
 *******************************************************************************/
uint16_t memoryQueue::size(void) { return block_vec_.size(); }