#pragma once
#ifndef _TEST_HPP_
#define _TEST_HPP_

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <w25n04kv.hpp>

class Test {
  public:
    Test(esp_log_level_t test_log_level);
    void testW25N04KV(spi_bus_config_t spi_cfg);

  private:
    void testReadDeviceStatus(void);

  private:
    W25N04KV spi_flash_;
};

#endif