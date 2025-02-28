#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <test.hpp>
#include <ad5626.hpp>
#include <ads1120.hpp>
#include <sensorSetup.hpp>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define ADC_VALUE_ERROR_MARGIN 0.003

static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testSensorSetup(void) {
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        return;        
    }

    gpio_config_t config;
    config.mode = GPIO_MODE_OUTPUT;
    config.intr_type = GPIO_INTR_DISABLE;
    config.pin_bit_mask = 1ULL << GPIO_NUM_17;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;

    ret = gpio_config(&config);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %d", ret);
        return;
    }

    ad5626_init_param_t dac_params;
    dac_params.cs_pin   = GPIO_NUM_0;
    dac_params.ldac_pin = GPIO_NUM_17;
    dac_params.clr_pin  = GPIO_NUM_NC;
    dac_params.spi_host = SPI2_HOST;

    // Initialize the ADC instance
    ads1120_init_param_t adc_params;
    adc_params.cs_pin   = GPIO_NUM_21;
    adc_params.drdy_pin = GPIO_NUM_2;
    adc_params.spi_host = SPI2_HOST;

    setup_.init(adc_params, dac_params);

    testSensorSetupZero();
    testSensorSetupReadAnalogFrontEnd();
}

void Test::testSensorSetupReadAnalogFrontEnd(void)
{
    ESP_LOGI(TAG, "Testing Reading Analog FrontEnd");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    sensor_measurement_t measurement;

    while (true) {
        setup_.measure(&measurement);
        ESP_LOGI(TAG, "Measurement: %f V, %d", measurement.voltage, measurement.adc_value);
        vTaskDelay(10);
    }
}

void Test::testSensorSetupZero(void)
{
    esp_err_t ret = setup_.zero();
    if (ret) {
        ESP_LOGI(TAG, "Failed to Zero");
        return;
    } else {
        ESP_LOGI(TAG, "Finished Zeroing");
    }
}