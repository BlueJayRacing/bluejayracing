#ifndef _MEMORY_QUEUE_HPP_
#define _MEMORY_QUEUE_HPP_

#include <esp_system.h>
#include <memoryBlock.hpp>

class memoryQueue {
  public:
    memoryQueue(const uint16_t t_num_blocks, const uint32_t t_block_size)
        : block_vec_(t_num_blocks, memoryBlock(t_block_size)), front_index_(0), back_index_(0), num_pushed_(0),
          acquired_lock_(false) {};
    memoryBlock* acquire(void);
    int8_t push(memoryBlock* t_mem_block);
    int8_t pop(memoryBlock& copy_block);
    uint16_t getNumPushed(void);
    uint16_t size(void);

  private:
    std::vector<memoryBlock> block_vec_;
    uint16_t front_index_;
    uint16_t back_index_;
    uint16_t num_pushed_;
    bool acquired_lock_;
};

#endif