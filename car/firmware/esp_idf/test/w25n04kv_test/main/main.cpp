#include <assert.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <stdio.h>
#include <test.hpp>
#include <cstring>

// Tested on 2025 WSG v1.0
#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

static const char* TAG = "main";

extern "C" void app_main(void)
{
    Test test(ESP_LOG_DEBUG);

    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    test.testW25N04KV(spi_cfg);
}