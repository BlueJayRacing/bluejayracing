#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>

static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testADS1120(void)
{
    ads1120_init_param_t adc_params;
    adc_params.cs_pin = GPIO_NUM_4;
    adc_params.drdy_pin = GPIO_NUM_3;
    adc_params.spi_host = SPI2_HOST;

    esp_err_t ret = adc_.init(adc_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADS1120: %d", ret);
        return;
    }

    testConfigureADS1120();
    testReadADS1120();
}

void Test::testConfigureADS1120(void)
{
    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    ads1120_regs_t readout_regs;

    adc_regs.analog_channels = 0x01;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully changed analog channels");

    adc_regs.volt_refs = 0x02;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set voltage references");

    adc_regs.gain = 0x04;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set gain");

    adc_regs.pga_bypass = true;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set PGA bypass");

    adc_regs.data_rate = 0x06;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set data rate");

    adc_regs.op_mode = 0x02;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set op mode");

    adc_regs.conv_mode = 0x01;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set conv mode");

    adc_regs.temp_mode = 0x01;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set temp mode");

    adc_regs.burn_sources = 0x01;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set current burnout sources");

    adc_regs.fir = 0x03;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set FIR");

    adc_regs.power_switch = 0x01;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set power switch");

    adc_regs.idac_current = 0x04;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set IDAC current");

    adc_regs.idac1_routing = 0x06;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set IDAC1 routing");

    adc_regs.idac2_routing = 0x04;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set IDAC2 routing");
    
    adc_regs.drdy_mode = 0x01;
    assert(adc_.configure(adc_regs) == ESP_OK);
    adc_.getRegs(&readout_regs);
    assert(memcmp(&adc_regs, &readout_regs, sizeof(ads1120_regs_t)) == 0);
    ESP_LOGI(TAG, "Successfully set DRDY mode");
}

void Test::testReadADS1120(void)
{

}