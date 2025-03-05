#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <ads1120.hpp>

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testADS1120(void);

  private:
    void testConfigureADS1120(void);
    void testSingleReadADS1120(void);
    void testContinuousReadADS1120(void);
    void testContinousReadViaInterruptADS1120(void);
    static void IRAM_ATTR ADCReadInterrupt(void* adc);

  private:
    ADS1120 adc_;
};

#endif