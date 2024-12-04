#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <memoryBlock.hpp>
#include <memoryQueue.hpp>
#include <mqttManager.hpp>

class Test {
  public:
    void testMemoryQueue(void);

  private:
    void testMemoryQueueBasic(void);
    void testMemoryQueueAcquireFull(void);
};

#endif