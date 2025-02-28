#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <sensorSetup.hpp>

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testSensorSetup(void);

  public:
    void testSensorSetupReadAnalogFrontEnd(void);
    void testSensorSetupZero(void);

  private:
    sensorSetup setup_;
};

#endif