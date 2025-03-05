#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testClass(void);
    // add public test functions here

  private:
    void testClassCase(void);
    // Add private test functions here

  private:
    // Add private variables here
};

#endif