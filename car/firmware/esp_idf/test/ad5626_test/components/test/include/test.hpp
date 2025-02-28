#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <ad5626.hpp>

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testAD5626(void);
    // add public test functions here

  private:
    void testAD5626Set(void);
    void testAD5626ParamErrors(void);
    // Add private test functions here

  private:
    AD5626 dac_;
};

#endif