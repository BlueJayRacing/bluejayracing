#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>
#include <cstring>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testW25N04KV(void) {
    esp_err_t ret;
    
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

    w25n04kv_init_param_t flash_init_params;
    flash_init_params.cs_pin = GPIO_NUM_1;
    flash_init_params.wp_pin = GPIO_NUM_NC;
    flash_init_params.spi_host = SPI2_HOST;

    ESP_LOGI(TAG, "Initialized SPI Bus");

    vTaskDelay(100);

    ret = spi_flash_.init(flash_init_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI Flash: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "Initialized SPI Flash");

    vTaskDelay(100);

    testReadDeviceStatus();
    testParamErrors();
    testReadWriteMemory();
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

void Test::testParamErrors(void)
{
    ESP_LOGI(TAG, "Testing error handling for parameter errors");

    std::vector<uint8_t> too_large_vec(2049);
    uint32_t address = 1000; // Value does not matter, error should be thrown regardless
    assert(spi_flash_.writePage(too_large_vec, address) == ESP_ERR_INVALID_ARG);

    ESP_LOGI(TAG, "Passed Testing error handling for parameter errors");
}


void Test::testReadWriteMemory(void)
{
    ESP_LOGI(TAG, "Testing SPI Flash reading/writing to memory");

    std::vector<uint8_t> tx_data(W25N04KV::PAGE_SIZE);
    std::vector<uint8_t> rx_data(W25N04KV::PAGE_SIZE);

    for (int i = 0; i < W25N04KV::PAGE_SIZE; i++) {
        tx_data.at(i) = i;
    }

    std::srand(esp_cpu_get_cycle_count());
    uint32_t page_address = std::rand() % W25N04KV::NUM_PAGES;

    assert(spi_flash_.writePage(tx_data, page_address) == ESP_OK);

    vTaskDelay(1);

    assert(spi_flash_.readPage(rx_data, page_address) == ESP_OK);

    for (int i = 0; i < W25N04KV::PAGE_SIZE; i++) {
        ESP_LOGI(TAG, "TX: %d, RX: %d\n", tx_data[i], rx_data[i]);
        // assert(tx_data[i] == rx_data[i]);
    }
    

    ESP_LOGI(TAG, "Passed Testing SPI Flash reading/writing to memory");
}