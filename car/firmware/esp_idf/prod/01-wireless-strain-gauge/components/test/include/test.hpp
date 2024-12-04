#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <memoryBlock.hpp>
#include <memoryQueue.hpp>
#include <mqttManager.hpp>

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testMemoryQueue(void);
    void testMQTTManager(void);

  private:
    void testMemoryQueueBasic(void);
    void testMemoryQueueAcquireFull(void);
    void testMQTTManagerWiFiConnectDisconnect(void);
    void testMQTTManagerMQTTConnectDisconnect(void);

  private:
    mqttManager mqtt_manager_;
};

#endif