#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <memoryBlock.hpp>
#include <memoryQueue.hpp>
#include <mqttManager.hpp>
#include <ad5626.hpp>
#include <ads1120.hpp>

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testMemoryQueue(void);
    void testMQTTManager(void);
    void testADCDAC(void);

  private:
    void testMemoryQueueBasic(void);
    void testMemoryQueueAcquireFull(void);

    void testMQTTManagerBasicParamErrors(void);
    void testMQTTManagerWiFiConnectDisconnect(void);
    void testMQTTManagerClientConnectDisconnect(void);
    void testMQTTManagerClientWiFiConnectDisconnect(void);
    void testMQTTManagerMultipleClientsConDisCon(void);
    void testMQTTManagerClientPublishSubscribe(void);

    void testADCDACCheckSPIBus(void);
    void testADCDACReadDACBias(void);
    void testADCDACReadAnalogFrontEnd(void);

  private:
    mqttManager* mqtt_manager_;
    AD5626 dac_;
    ADS1120 adc_;
};

#endif