#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>
#include <cstring>


static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testW25N04KV(spi_bus_config_t spi_cfg) { // Configure the SPI bus
    esp_err_t ret;

    spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

    w25n04kv_init_param_t flash_init_params;
    flash_init_params.cs_pin = GPIO_NUM_1;
    flash_init_params.wp_pin = GPIO_NUM_NC;
    flash_init_params.spi_host = SPI2_HOST;

    ESP_LOGI(TAG, "Initialized SPI Bus");

    ret = spi_flash_.init(flash_init_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI Flash: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "Initialized SPI Flash");

    vTaskDelay(100);

    testReadDeviceStatus();
}

void Test::testReadDeviceStatus(void)
{
    ESP_LOGI(TAG, "Testing SPI Flash Reading Device ID and initial status");

    assert(spi_flash_.isCorrectDevice() == ESP_OK);

    w25n04kv_device_status_t dev_status;

    assert(spi_flash_.readStatus(&dev_status) == ESP_OK);

    assert(dev_status.ecc_status == NO_ERROR);
    assert(dev_status.erase_failure == false);
    assert(dev_status.is_busy == false);
    assert(dev_status.program_failure == false);
    assert(dev_status.write_enable == false);

    ESP_LOGI(TAG, "Passed Testing SPI Flash Reading Device ID and initial status");
}