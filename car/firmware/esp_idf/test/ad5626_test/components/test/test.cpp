#include "freertos/FreeRTOS.h"
#include <ad5626.hpp>
#include <cstring>
#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>

static const char* TAG = "test";

#define SPI_MOSI_PIN 18
#define SPI_MISO_PIN 20
#define SPI_SCLK_PIN 19

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testAD5626(void)
{
    // Need to connect to DAC output in order to see if it works
    ad5626_init_param_t dac_params;
    dac_params.cs_pin   = GPIO_NUM_0;
    dac_params.ldac_pin = GPIO_NUM_23;
    dac_params.clr_pin  = GPIO_NUM_NC;
    dac_params.spi_host = SPI2_HOST;

    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        return;
    }

    ret = dac_.init(dac_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AD5626: %d", ret);
        return;
    }

    testAD5626Set();
    testAD5626ParamErrors();
}

#define NUM_VALUES            16

void Test::testAD5626Set(void)
{
    ESP_LOGI(TAG, "Testing setting different values on DAC");

    int dac_value = 0;
    int dac_incr  = AD5626::MAX_LEVEL_VALUE / NUM_VALUES;

    for (int i = 0; i <= NUM_VALUES; i++) {
        dac_.setLevel(dac_value);
        vTaskDelay(50);
        dac_value += dac_incr;
    }

    ESP_LOGI(TAG, "Passed setting values on DAC");
}

void Test::testAD5626ParamErrors(void) {
    ESP_LOGI(TAG, "Testing DAC driver error handling");

    assert(dac_.setLevel(AD5626::MAX_LEVEL_VALUE + 1) == ESP_ERR_INVALID_ARG);

    ESP_LOGI(TAG, "Finished testing DAC driver error handling");
}
