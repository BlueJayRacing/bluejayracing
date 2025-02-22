#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>
#include <atomic>
#include "esp_timer.h"

static const char* TAG = "test";

#define SPI_MOSI_PIN    18
#define SPI_MISO_PIN    20
#define SPI_SCLK_PIN    19

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

// Tested with WSG v2.0 board as labeled physically (could be labeled as v1.0 on KiCAD)
void Test::testADS1120(void)
{
    gpio_install_isr_service(0);

    ads1120_init_param_t adc_params;
    adc_params.cs_pin = GPIO_NUM_21;
    adc_params.drdy_pin = GPIO_NUM_2;
    adc_params.spi_host = SPI2_HOST;

    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num = SPI_MOSI_PIN;
    spi_cfg.miso_io_num = SPI_MISO_PIN;
    spi_cfg.sclk_io_num = SPI_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        return;
    }

    ret = adc_.init(adc_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADS1120: %d", ret);
        return;
    }

    testConfigureADS1120();
    testSingleReadADS1120();
    testContinuousReadADS1120();
    testContinousReadViaInterruptADS1120();
}

void Test::testConfigureADS1120(void)
{
    ESP_LOGI(TAG, "Testing Configuring ADC");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    ads1120_regs_t readout_regs;

    adc_regs.channels = AIN0_AIN1;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully changed analog channels");

    adc_regs.volt_refs = REFP0_REFN0;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set voltage references");

    adc_regs.gain = GAIN_32;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set gain");

    adc_regs.pga_bypass = true;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set PGA bypass");

    adc_regs.data_rate = 0x06;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set data rate");

    adc_regs.op_mode = TURBO;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set op mode");

    adc_regs.conv_mode = CONTINUOUS;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set conv mode");

    adc_regs.temp_mode = TEMPMODE_ENABLED;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set temp mode");

    adc_regs.burn_sources = true;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set current burnout sources");

    adc_regs.fir = REJ_60HZ;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set FIR");

    adc_regs.power_switch = true;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set power switch");

    adc_regs.idac_current = IDAC_ON_1500uA;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set IDAC current");

    adc_regs.idac1_routing = REFN0;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set IDAC1 routing");

    adc_regs.idac2_routing = REFP0;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set IDAC2 routing");
    
    adc_regs.drdy_mode = DRDY_DOUT;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGV(TAG, "Successfully set DRDY mode");

    ESP_LOGI(TAG, "Passed configuring ADC");
}

#define NUM_SINGLE_SAMPLES 10

void Test::testSingleReadADS1120(void)
{
    ESP_LOGI(TAG, "Testing Single Reading ADC");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = SINGLE_SHOT;

    adc_.configure(adc_regs);

    int16_t data;
    int num_success_reads = 0;
    
    for (int i = 0; i < NUM_SINGLE_SAMPLES; i++) {
        if (adc_.readADC(&data) == ESP_OK) {
            num_success_reads++;
        }
    }    

    ESP_LOGI(TAG, "Number of successful reads: %d", num_success_reads);
    assert(num_success_reads == NUM_SINGLE_SAMPLES);

    ESP_LOGI(TAG, "Passed Single Testing Reading ADC");
}

void Test::testContinuousReadADS1120(void)
{
    ESP_LOGI(TAG, "Testing Continuous Reading ADC");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode = TURBO;
    adc_regs.data_rate = 6; // 2000 SPS

    adc_.configure(adc_regs);

    int16_t data;
    int num_success_reads = 0;
    int num_failed_reads = 0;
    int start_time = esp_timer_get_time();
    int current_time = esp_timer_get_time();

    while (current_time - 1000000 < start_time) {
        while (!adc_.isDataReady()) {}
        if (adc_.readADC(&data) == ESP_OK) {
            num_success_reads++;
        } else {
            num_failed_reads++;
        }

        current_time = esp_timer_get_time();
    }

    ESP_LOGI(TAG, "Number of successful reads: %d", num_success_reads);
    ESP_LOGI(TAG, "Number of failed reads: %d", num_failed_reads);
    assert(num_success_reads > 1900 && num_success_reads < 2100);

    ESP_LOGI(TAG, "Passed Continuous Testing Reading ADC");
}


std::atomic<bool> data_ready;
int num_success_reads = 0;
int num_failed_reads = 0;

void IRAM_ATTR Test::ADCReadInterrupt(void*) {
    BaseType_t woken;
    data_ready = true;
}

void Test::testContinousReadViaInterruptADS1120(void) {
    ESP_LOGI(TAG, "Testing Continuous Reading via Interrupt ADC");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode = TURBO;
    adc_regs.data_rate = 6; // 2000 SPS

    adc_.configure(adc_regs);

    int start_time = esp_timer_get_time();
    int current_time = esp_timer_get_time();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // Interrupt on rising edge
    };
    
    gpio_config(&io_conf);

    esp_err_t ret = gpio_isr_handler_add(GPIO_NUM_2, Test::ADCReadInterrupt, NULL);
    if (ret) {
        return;
    }

    int16_t data;

    while (current_time - 1000000 < start_time) {
        while (data_ready == false) {};
        data_ready = false;

        ret = adc_.readADC(&data);
        if (ret) {
            num_failed_reads++;
        } else {
            num_success_reads++;
        }

        current_time = esp_timer_get_time();
    }

    ret = gpio_isr_handler_remove(GPIO_NUM_2);
    if (ret) {
        return;
    }

    ESP_LOGI(TAG, "Number of successful reads: %d", num_success_reads);
    ESP_LOGI(TAG, "Number of failed reads: %d", num_failed_reads);
    
    assert(num_success_reads > 1900 && num_success_reads < 2100);

    ESP_LOGI(TAG, "Testing Continuous Reading via Interrupt ADC");
}