#pragma once
#ifndef _MEMORY_BLOCK_HPP_
#define _MEMORY_BLOCK_HPP_

#include <esp_system.h>
#include <stdio.h>
#include <vector>

class memoryBlock {
  public:
    memoryBlock(const int32_t t_size) : data_(t_size, 0), index_(0) {};
    int32_t write(const std::vector<uint8_t>& t_new_data);
    uint8_t* data(void);
    int32_t size(void);
    int32_t bytesLeft(void);
    void clear(void);

  private:
    std::vector<uint8_t> data_;
    int32_t index_;
};

#endif