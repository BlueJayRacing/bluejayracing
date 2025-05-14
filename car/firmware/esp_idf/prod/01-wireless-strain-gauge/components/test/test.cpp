#include <config.hpp>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <test.hpp>
#include <w25n04kv.hpp>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define ADC_VALUE_ERROR_MARGIN 0.003

#include <hardEncoder.hpp>
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <wsg_drive_data.pb.h>

#if ENABLE_TESTS == 1

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define ADC_VALUE_ERROR_MARGIN 0.006

static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testMQTTManager(void)
{
    esp_log_level_set("wifi", ESP_LOG_NONE);
    esp_log_level_set("wifi_init", ESP_LOG_NONE);

    mqtt_manager_ = mqttManager::getInstance();

    // Initialize the MQTT Manager
    assert(mqtt_manager_->init("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
    ESP_LOGD(TAG, "Initialized MQTT manager");

    // Run MQTT manager tests
    testMQTTManagerBasicParamErrors();
    testMQTTManagerWiFiConnectDisconnect();
    testMQTTManagerClientConnectDisconnect();
    testMQTTManagerClientWiFiConnectDisconnect();
    testMQTTManagerMultipleClientsConDisCon();
    testMQTTManagerClientPublishSubscribe();
}

/* We check to see if the MQTT manager returns the correct return values on error.
 */
void Test::testMQTTManagerBasicParamErrors(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager Basic Paramter Error Handling");

    mqtt_client_t client;

    assert(mqtt_manager_->createClient("") == NULL);

    assert(mqtt_manager_->clientConnect(NULL) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientConnect(&client) == ESP_ERR_WIFI_NOT_CONNECT); // Fail because WiFi not connected

    assert(mqtt_manager_->clientDisconnect(NULL) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->isClientConnected(NULL) == false);

    std::string test_mes("hi");

    assert(mqtt_manager_->clientPublish(NULL, NULL, 5, "topic", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientPublish(&client, (uint8_t*)test_mes.data(), 0, "topic", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientPublish(&client, (uint8_t*)test_mes.data(), test_mes.length(), "", 2) ==
           ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientPublish(&client, (uint8_t*)test_mes.data(), test_mes.length(), "topic", 3) ==
           ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientPublish(&client, (uint8_t*)test_mes.data(), test_mes.length(), "topic", 2) ==
           ESP_ERR_WIFI_NOT_CONNECT);

    assert(mqtt_manager_->clientSubscribe(NULL, "topic", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientSubscribe(&client, "", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientSubscribe(&client, "topic", 3) == ESP_ERR_INVALID_ARG);

    mqtt_message_t message;
    assert(mqtt_manager_->clientReceive(NULL, message, 100) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientClearMessages(NULL) == ESP_ERR_INVALID_ARG);

    ESP_LOGI(TAG, "Passed MQTT Manager Basic Paramter Error Handling");
}

/* We check to see if the MQTT manager can successfully connect/disconnect from WiFi,
 * as well as properly signal the WiFi connection event group.
 */
void Test::testMQTTManagerWiFiConnectDisconnect(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager WiFi Connection");

    for (int i = 0; i < 5; i++) {
        assert(mqtt_manager_->connectWiFi() == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");

        mqtt_manager_->disconnectWiFi();

        for (int j = 0; j < 5; j++) {
            if (!mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
        vTaskDelay(100);
    }

    ESP_LOGI(TAG, "Passed MQTT Manager WiFi Connection");
}

/* We check to see if the MQTT manager can successfully connect/disconnect from MQTT,
 * as well as properly signal the MQTT connection event group.
 */
void Test::testMQTTManagerClientConnectDisconnect(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager MQTT Connection");

    // Initialization (Connects to WiFi)
    {
        assert(mqtt_manager_->connectWiFi() == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");
    }

    mqtt_client_t* client = mqtt_manager_->createClient("mqtt://10.42.0.1");
    assert(client != NULL);
    assert(mqtt_manager_->isClientConnected(client) == false);

    for (int i = 0; i < 5; i++) {
        assert(mqtt_manager_->clientConnect(client) == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to MQTT");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_->isClientConnected(client)) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isClientConnected(client) == true);
        ESP_LOGD(TAG, "Connected to MQTT");

        mqtt_manager_->clientDisconnect(client);

        for (int j = 0; j < 20; j++) {
            if (!mqtt_manager_->isClientConnected(client)) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isClientConnected(client) == false);
        ESP_LOGD(TAG, "Disconnected from MQTT");
        vTaskDelay(300);
    }

    mqtt_manager_->destroyClient(client);

    // Deinitialization (Discnnects from WiFi)
    {
        mqtt_manager_->disconnectWiFi();

        for (int i = 0; i < 5; i++) {
            if (!mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
    }

    ESP_LOGI(TAG, "Passed MQTT Manager MQTT Connection");
}

/* We check to see if the MQTT manager can properly signal an MQTT disconnection if the WiFi
 * connection breaks down.
 */
void Test::testMQTTManagerClientWiFiConnectDisconnect(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager MQTT + WIFI Disconnect");

    mqtt_client_t* client = mqtt_manager_->createClient("mqtt://10.42.0.1");
    assert(client != NULL);

    // Initialization (Connects to WiFi + MQTT)
    {
        assert(mqtt_manager_->connectWiFi() == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_->isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");

        assert(mqtt_manager_->clientConnect(client) == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to MQTT");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_->isClientConnected(client)) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_->isClientConnected(client) == true);
        ESP_LOGD(TAG, "Connected to MQTT");
    }

    mqtt_manager_->disconnectWiFi();

    for (int j = 0; j < 5; j++) {
        if (!mqtt_manager_->isWiFiConnected()) {
            break;
        }
        vTaskDelay(100);
    }

    assert(mqtt_manager_->isWiFiConnected() == false);
    assert(mqtt_manager_->isClientConnected(client) == false);

    mqtt_manager_->destroyClient(client);

    ESP_LOGD(TAG, "Disconnected from WiFi and MQTT");

    ESP_LOGI(TAG, "Passed MQTT Manager MQTT + WIFI Disconnect");
}

/* We check to see if the MQTT manager can properly handle connecting and disconnecting multiple
 * MQTT clients.
 */
void Test::testMQTTManagerMultipleClientsConDisCon(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager Multiple Clients Connect/Disconnect");

    {
        assert(mqtt_manager_->connectWiFi() == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_->isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");
    }

    mqtt_client_t* client_1 = mqtt_manager_->createClient("mqtt://10.42.0.1");
    assert(client_1 != NULL);

    mqtt_client_t* client_2 = mqtt_manager_->createClient("mqtt://10.42.0.1");
    assert(client_2 != NULL);
    ESP_LOGD(TAG, "Created MQTT Clients 1 and 2");

    assert(mqtt_manager_->clientConnect(client_1) == ESP_OK);
    ESP_LOGD(TAG, "Client 1 Started Connecting to MQTT");

    for (int j = 0; j < 30; j++) {
        if (mqtt_manager_->isClientConnected(client_1)) {
            break;
        }
        vTaskDelay(50);
    }

    assert(mqtt_manager_->isClientConnected(client_1) == true);
    assert(mqtt_manager_->isClientConnected(client_2) == false);
    ESP_LOGD(TAG, "Client 1 Connected to MQTT but not Client 2");

    assert(mqtt_manager_->clientConnect(client_2) == ESP_OK);
    ESP_LOGD(TAG, "Client 2 Started Connecting to MQTT");

    for (int j = 0; j < 30; j++) {
        if (mqtt_manager_->isClientConnected(client_2)) {
            break;
        }
        vTaskDelay(50);
    }

    assert(mqtt_manager_->isClientConnected(client_1) == true);
    assert(mqtt_manager_->isClientConnected(client_2) == true);
    ESP_LOGD(TAG, "Client 1 and 2 Connected to MQTT");

    mqtt_manager_->clientDisconnect(client_1);

    for (int j = 0; j < 20; j++) {
        if (!mqtt_manager_->isClientConnected(client_1)) {
            break;
        }
        vTaskDelay(50);
    }

    assert(mqtt_manager_->isClientConnected(client_1) == false);
    assert(mqtt_manager_->isClientConnected(client_2) == true);
    ESP_LOGD(TAG, "Client 1 Disconnected from MQTT and Client 2 Connected");

    mqtt_manager_->clientDisconnect(client_2);

    for (int j = 0; j < 20; j++) {
        if (!mqtt_manager_->isClientConnected(client_2)) {
            break;
        }
        vTaskDelay(50);
    }

    assert(mqtt_manager_->isClientConnected(client_1) == false);
    assert(mqtt_manager_->isClientConnected(client_2) == false);
    ESP_LOGD(TAG, "Client 1 and 2 Disconnected from MQTT");

    mqtt_manager_->destroyClient(client_1);
    mqtt_manager_->destroyClient(client_2);

    // Deinitialization (Discnnects from WiFi)
    {
        mqtt_manager_->disconnectWiFi();

        for (int i = 0; i < 5; i++) {
            if (!mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
    }

    ESP_LOGI(TAG, "Passed MQTT Manager Multiple Clients Connect/Disconnect");
}

/* We check to see if the MQTT manager can properly publish and subscribe to topics from the MQTT broker.
 */
void Test::testMQTTManagerClientPublishSubscribe(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager Client Publish/Subscribe");

    {
        assert(mqtt_manager_->connectWiFi() == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_->isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");
    }

    mqtt_client_t* client = mqtt_manager_->createClient("mqtt://10.42.0.1");
    assert(client != NULL);

    assert(mqtt_manager_->clientConnect(client) == ESP_OK);
    ESP_LOGD(TAG, "Client Started Connecting to MQTT");

    for (int j = 0; j < 30; j++) {
        if (mqtt_manager_->isClientConnected(client)) {
            break;
        }
        vTaskDelay(50);
    }

    assert(mqtt_manager_->isClientConnected(client) == true);
    ESP_LOGD(TAG, "Client Connected to MQTT");

    assert(mqtt_manager_->clientSubscribe(client, "esp32/test_subscribe", 2) == ESP_OK);
    assert(mqtt_manager_->clientWaitSubscribe(client, 1000) == ESP_OK);
    assert(mqtt_manager_->clientWaitSubscribe(client, 10) == ESP_ERR_TIMEOUT);
    ESP_LOGD(TAG, "Client Subscribed to MQTT");

    mqtt_message_t rec_mes;

    for (int i = 0; i < 10; i++) {
        std::string hi_mes("hi " + std::to_string(i));
        assert(mqtt_manager_->clientPublish(client, (uint8_t*)hi_mes.data(), hi_mes.length(), "esp32/test_publish",
                                            2) == ESP_OK);
        // assert(mqtt_manager_->clientWaitPublish(client, 1000) == ESP_OK);
        // assert(mqtt_manager_->clientWaitPublish(client, 10) == ESP_ERR_TIMEOUT);
        ESP_LOGD(TAG, "Client Published to MQTT");

        memset(&rec_mes, 0, sizeof(mqtt_message_t));

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_->clientReceive(client, rec_mes, 100) == ESP_OK) {
                ESP_LOGD(TAG, "Received topic: %s", rec_mes.topic.data());
                ESP_LOGD(TAG, "Received data: %s", rec_mes.payload.data());
                break;
            }
            vTaskDelay(50);
        }
    }

    mqtt_manager_->destroyClient(client);

    // Deinitialization (Discnnects from WiFi)
    {
        mqtt_manager_->disconnectWiFi();

        for (int i = 0; i < 5; i++) {
            if (!mqtt_manager_->isWiFiConnected()) {
                break;
            }
            vTaskDelay(50);
        }

        assert(mqtt_manager_->isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
    }

    ESP_LOGI(TAG, "Passed MQTT Manager Client Publish/Subscribe");
}

void Test::testADCDACEndtoEnd(void)
{
    gpio_config_t config;
    config.mode         = GPIO_MODE_OUTPUT;
    config.intr_type    = GPIO_INTR_DISABLE;
    config.pin_bit_mask = 1ULL << GPIO_NUM_17;
    config.pull_up_en   = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;

    esp_err_t ret = gpio_config(&config);
    if (ret) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %d", ret);
        return;
    }

    // Configure the SPI bus
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);

    // Initialize the DAC instance
    ad5626_init_param_t dac_params;
    dac_params.cs_pin   = GPIO_NUM_0;
    dac_params.ldac_pin = GPIO_NUM_17;
    dac_params.clr_pin  = GPIO_NUM_NC;
    dac_params.spi_host = SPI2_HOST;

    ret = dac_.init(dac_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AD5626: %d", ret);
        return;
    }

    // Initialize the ADC instance
    ads1120_init_param_t adc_params;
    adc_params.cs_pin   = GPIO_NUM_21;
    adc_params.drdy_pin = GPIO_NUM_2;
    adc_params.spi_host = SPI2_HOST;

    ret = adc_.init(adc_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADS1120: %d", ret);
        return;
    }

    // Start running ADC/DAC tests
    testADCDACCheckSPIBus();
    testADCDACReadDACBias();
    testADCDACTestADCGain2();
    testADCDACTestADCGain4();
}

/* We check whether the SPI bus on the WSG board is functional by programming
 * the ADC to read data at a specific sample rate, and then check to see if the
 * programmed sample rate is close to the real sample rate.
 */

void Test::testADCDACCheckSPIBus(void)
{
    ESP_LOGI(TAG, "Testing Reading ADC to check SPI bus");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode   = TURBO; // turbo
    adc_regs.data_rate = 6;     // 2000 SPS

    adc_.configure(adc_regs);

    int16_t data;
    int num_success_reads = 0;
    int num_failed_reads  = 0;
    int start_time        = esp_timer_get_time();
    int current_time      = esp_timer_get_time();

    while (current_time - 1000000 < start_time) {
        if (adc_.isDataReady()) {
            if (adc_.readADC(&data) == ESP_OK) {
                num_success_reads++;
            } else {
                num_failed_reads++;
            }
        }
        current_time = esp_timer_get_time();
    }

    ESP_LOGD(TAG, "Number of successful reads: %d", num_success_reads);
    ESP_LOGD(TAG, "Number of failed reads: %d", num_failed_reads);
    assert(num_success_reads > 1800 && num_success_reads < 2200);

    ESP_LOGI(TAG, "Passed Testing Reading ADC to check SPI bus");
}

#define NUM_DAC_BIAS_SAMPLES 315

/* We check whether the DAC bias is being set properly by having the ADC read it
 * as a single-ended input between the DAC bias and ground.
 */
void Test::testADCDACReadDACBias(void)
{
    ESP_LOGI(TAG, "Testing Reading DAC Bias from ADC");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode   = TURBO;
    adc_regs.channels  = AIN2_AVSS;
    adc_regs.data_rate = 6;
    adc_regs.volt_refs = REFP0_REFN0;

    esp_err_t ret = adc_.configure(adc_regs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %d", ret);
        return;
    }

    float sum_diff = 0;

    for (int i = 0; i < NUM_DAC_BIAS_SAMPLES; i++) {
        ret = dac_.setLevel(AD5626::MAX_LEVEL_VALUE * i / (NUM_DAC_BIAS_SAMPLES));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set DAC level: %d", ret);
            continue;
        }

        vTaskDelay(1);

        while (!adc_.isDataReady()) {
            vTaskDelay(1);
        }

        int16_t read_value;

        ret = adc_.readADC(&read_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC: %d", ret);
            continue;
        }

        float expected_dac_voltage = 4.095 * i / (NUM_DAC_BIAS_SAMPLES);
        float measured_dac_voltage = 4.999 * read_value / ((1 << 15) - 1);
        sum_diff += std::abs(expected_dac_voltage - measured_dac_voltage);

        ESP_LOGD(TAG, "Expected DAC Voltage Value: %.10f", expected_dac_voltage);
        ESP_LOGD(TAG, "Measured DAC Voltage Value: %.10f", measured_dac_voltage);
    }

    ESP_LOGI(TAG, "Average error (V): %f", sum_diff / NUM_DAC_BIAS_SAMPLES);

    assert(sum_diff / NUM_DAC_BIAS_SAMPLES < ADC_VALUE_ERROR_MARGIN);

    ESP_LOGI(TAG, "Passed reading DAC Bias from ADC");
}

#define NUM_ADC_GAIN2_SAMPLES 250
#define MVOLTS_PER_SAMPLE     10

/* We check whether the ADC gain is being correctly set by changing the
 * gain to a predetermined DAC bias.
 */
void Test::testADCDACTestADCGain2(void)
{
    ESP_LOGI(TAG, "Testing ADC gain 2");

    esp_err_t ret;
    ads1120_regs_t adc_regs;
    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode   = TURBO;
    adc_regs.channels  = AIN2_AVSS;
    adc_regs.data_rate = 6;
    adc_regs.volt_refs = REFP0_REFN0;
    adc_regs.gain      = GAIN_2;

    ret = adc_.configure(adc_regs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %d", ret);
    }

    float sum_diff = 0;

    for (int i = 0; i < NUM_ADC_GAIN2_SAMPLES; i++) {

        ret = dac_.setLevel(i * MVOLTS_PER_SAMPLE);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "Failed to set DAC Level: %d", ret);
            continue;
        }

        vTaskDelay(1);

        while (!adc_.isDataReady()) {
            vTaskDelay(1);
        }

        int16_t read_value;

        ret = adc_.readADC(&read_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC: %d", ret);
            continue;
        }

        float expected_dac_voltage = 0.001 * i * MVOLTS_PER_SAMPLE * 2;
        float measured_dac_voltage = 4.999 * read_value / ((1 << 15) - 1);
        sum_diff += std::abs(expected_dac_voltage - measured_dac_voltage);

        ESP_LOGD(TAG, "Expected DAC Voltage Value: %.10f", expected_dac_voltage);
        ESP_LOGD(TAG, "Measured DAC Voltage Value: %.10f", measured_dac_voltage);
    }

    ESP_LOGI(TAG, "Average error (V): %f", sum_diff / NUM_ADC_GAIN2_SAMPLES);

    ESP_LOGI(TAG, "Finished testing ADC gain 2");
}

#define NUM_ADC_GAIN4_SAMPLES 125
#define MVOLTS_PER_SAMPLE     10

void Test::testADCDACTestADCGain4(void)
{

    ESP_LOGI(TAG, "Testing ADC gain 4");

    esp_err_t ret;
    ads1120_regs_t adc_regs;
    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode   = TURBO;
    adc_regs.channels  = AIN2_AVSS;
    adc_regs.data_rate = 6;
    adc_regs.volt_refs = REFP0_REFN0;
    adc_regs.gain      = GAIN_4;

    ret = adc_.configure(adc_regs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %d", ret);
    }

    float sum_diff = 0;

    for (int i = 0; i < NUM_ADC_GAIN4_SAMPLES; i++) {

        ret = dac_.setLevel(i * MVOLTS_PER_SAMPLE);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "Failed to set DAC Level: %d", ret);
            continue;
        }

        vTaskDelay(1);

        while (!adc_.isDataReady()) {
            vTaskDelay(1);
        }

        int16_t read_value;

        ret = adc_.readADC(&read_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC: %d", ret);
            continue;
        }

        float expected_dac_voltage = 0.001 * i * MVOLTS_PER_SAMPLE * 4;
        float measured_dac_voltage = 4.999 * read_value / ((1 << 15) - 1);
        sum_diff += std::abs(expected_dac_voltage - measured_dac_voltage);

        ESP_LOGD(TAG, "Expected DAC Voltage Value: %.10f", expected_dac_voltage);
        ESP_LOGD(TAG, "Measured DAC Voltage Value: %.10f", measured_dac_voltage);
    }

    ESP_LOGI(TAG, "Average error (V): %f", sum_diff / NUM_ADC_GAIN4_SAMPLES);

    ESP_LOGI(TAG, "Finished testing ADC gain 4");
}

/**
 * We try to read the analog frontend (i.e. the filtered signal from the inst. amplifier)
 * to confirm that the numbers we see are reasonable.
 */
void Test::testADCDACReadAnalogFrontEnd(void)
{
    ESP_LOGI(TAG, "Testing Reading Analog FrontEnd");

    // Configure the SPI bus
    esp_err_t ret;
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (ret) {
        return;
    }

    // Initialize the DAC instance
    ad5626_init_param_t dac_params;
    dac_params.cs_pin   = GPIO_NUM_0;
    dac_params.ldac_pin = GPIO_NUM_17;
    dac_params.clr_pin  = GPIO_NUM_NC;
    dac_params.spi_host = SPI2_HOST;

    ret = dac_.init(dac_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AD5626: %d", ret);
        return;
    }

    // Initialize the ADC instance
    ads1120_init_param_t adc_params;
    adc_params.cs_pin   = GPIO_NUM_21;
    adc_params.drdy_pin = GPIO_NUM_2;
    adc_params.spi_host = SPI2_HOST;

    ret = adc_.init(adc_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADS1120: %d", ret);
        return;
    }

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    // Sample at Normal Mode, Gain of 1
    adc_regs.conv_mode = CONTINUOUS;
    adc_regs.op_mode   = NORMAL;
    adc_regs.channels  = AIN1_AIN2;
    adc_regs.data_rate = 2;
    adc_regs.volt_refs = REFP0_REFN0;
    adc_regs.gain      = GAIN_1;

    ret = adc_.configure(adc_regs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %d", ret);
        return;
    }

    ret = dac_.setLevel(2500);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DAC to ground: %d", ret);
        return;
    }

    int16_t adc_val;

    while (true) {
        if (adc_.isDataReady()) {
            ret = adc_.readADC(&adc_val);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read ADC: %d", ret);
                continue;
            }

            printf("%d\n", adc_val);
        } else {
            vTaskDelay(1);
        }
    }
}

void Test::testCalSensorSetup(void)
{
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
    config.mode         = GPIO_MODE_OUTPUT;
    config.intr_type    = GPIO_INTR_DISABLE;
    config.pin_bit_mask = 1ULL << GPIO_NUM_17;
    config.pull_up_en   = GPIO_PULLUP_DISABLE;
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

    cal_setup_.init(adc_params, dac_params);

    testCalSensorSetupZero();
    testCalSensorSetupReadAnalogFrontEnd();
}

void Test::testCalSensorSetupReadAnalogFrontEnd(void)
{
    ESP_LOGI(TAG, "Testing Reading Calibration Analog FrontEnd");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    cal_measurement_t measurement;

    while (true) {
        cal_setup_.measure(&measurement);
        ESP_LOGI(TAG, "Measurement: %f V, %d", measurement.voltage, measurement.adc_value);
        vTaskDelay(10);
    }
}

void Test::testCalSensorSetupZero(void)
{
    ESP_LOGI(TAG, "Testing Zeroing Calibration Sensor Setup");

    esp_err_t ret = cal_setup_.zero();
    if (ret) {
        ESP_LOGI(TAG, "Failed to Zero");
        return;
    } else {
        ESP_LOGI(TAG, "Finished Zeroing");
    }

    ESP_LOGI(TAG, "Finished Testing Zeroing Calibration Sensor Setup");
}

void Test::testDriveSensorSetup(void)
{
    ESP_LOGI(TAG, "Testing Zeroing Drive Sensor Setup");

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
    config.mode         = GPIO_MODE_OUTPUT;
    config.intr_type    = GPIO_INTR_DISABLE;
    config.pin_bit_mask = 1ULL << GPIO_NUM_17;
    config.pull_up_en   = GPIO_PULLUP_DISABLE;
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

    drive_setup_.init(adc_params, dac_params);

    testDriveSensorSetupSPS();
    testDriveSensorSetupZero();
    testDriveSensorSetupReadAnalogFrontEnd();

    ESP_LOGI(TAG, "Finished Testing Zeroing Drive Sensor Setup");
}

void Test::testProtobufEncode(void)
{
    testProtobufStockEncode();
    testProtobufHardEncode();
}

void Test::testDriveSensorSetupSPS(void)
{
    ESP_LOGI(TAG, "Testing Drive Sensor Setup SPS");

    uint32_t start_time = esp_timer_get_time();
    drive_measurement_t measurement;
    int num_success_reads = 0;

    while (esp_timer_get_time() - start_time < 1000000) {
        drive_setup_.measure(true, &measurement);
        num_success_reads++;
    }

    assert(num_success_reads > 1800 && num_success_reads < 2200);

    ESP_LOGI(TAG, "Passed Testing Drive Sensor Setup SPS");
}

void Test::testDriveSensorSetupReadAnalogFrontEnd(void)
{
    ESP_LOGI(TAG, "Testing Reading Drive Analog FrontEnd");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    drive_measurement_t measurement;

    drive_cfg_t sample_cfg  = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::STRAIN_GAUGE};
    drive_cfg_t excitn_cfg  = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::EXCITATION};
    drive_cfg_t dacbias_cfg = {drive_cfg_t::MEASURING_MODE, drive_cfg_t::DAC_BIAS};

    float strain_gauge_volt;
    float excitation_volt;
    float dac_bias_volt;

    while (true) {
        drive_setup_.configure(sample_cfg);
        drive_setup_.measure(true, &measurement);
        strain_gauge_volt = measurement.voltage;

        drive_setup_.configure(excitn_cfg);
        drive_setup_.measure(true, &measurement);
        excitation_volt = measurement.voltage;

        drive_setup_.configure(dacbias_cfg);
        drive_setup_.measure(true, &measurement);
        dac_bias_volt = measurement.voltage;

        ESP_LOGI(TAG, "%f \t%f \t%f", strain_gauge_volt, excitation_volt, dac_bias_volt);
        vTaskDelay(50);
    }
}

void Test::testDriveSensorSetupZero(void)
{
    ESP_LOGI(TAG, "Testing Zeroing Drive Sensor Setup");

    esp_err_t ret = drive_setup_.zero();
    if (ret) {
        ESP_LOGI(TAG, "Failed to Zero");
        return;
    } else {
        ESP_LOGI(TAG, "Finished Zeroing");
    }

    ESP_LOGI(TAG, "Finished Testing Zeroing Drive Sensor Setup");
}

#define NUM_ENCODES 100

std::array<uint8_t, 12000> buffer_1;
std::array<uint8_t, 12000> buffer_2;
wsg_drive_data_t measurements = wsg_drive_data_t_init_zero;
wsg_drive_data_t out_measurements = wsg_drive_data_t_init_zero;

void Test::testProtobufStockEncode(void)
{
    ESP_LOGI(TAG, "Testing Protobuf stock encoding");

    uint32_t encode_times_micros[NUM_ENCODES];
    uint32_t start_time;
    uint32_t end_time;
    pb_ostream_t o_stream;

    // Generate a test data packet to be serialized
    measurements.base_timestamp = esp_timer_get_time();
    measurements.dac_bias = 4095;
    measurements.excitation_voltage = 2.5;
    strcpy(measurements.mac_address, "AB:CD:EF:GH:IJ:KL");
    measurements.sample_channel_id = 0;

    for (int i = 0; i < NUM_SAMPLES_PER_MESSAGE; i++) {
        measurements.timestamp_deltas[i] = i * 500;
        measurements.values[i] = 3.000;
    }

    for (int i = 0; i < NUM_ENCODES; i++) {
        o_stream     = pb_ostream_from_buffer(buffer_1.data(), buffer_1.size());
    
        start_time = esp_timer_get_time();
        pb_encode(&o_stream, wsg_drive_data_t_fields, &measurements);
        end_time = esp_timer_get_time();

        encode_times_micros[i] = end_time - start_time;
    }

    uint64_t total_micros = 0;
    for (int i = 0; i < NUM_ENCODES; i++) {
        total_micros += encode_times_micros[i];
    }

    ESP_LOGI(TAG, "Average stock encoding time (micros): %f", ((float) total_micros) / NUM_ENCODES);
    ESP_LOGI(TAG, "Stock encoded bytestream length (bytes): %u", o_stream.bytes_written);
    ESP_LOGI(TAG, "Finished testing Protobuf stock encoding");
}

void Test::testProtobufHardEncode(void) {
    ESP_LOGI(TAG, "Testing Protobuf hard encoding");
    
    uint32_t encode_times_micros[NUM_ENCODES];
    uint32_t start_time;
    uint32_t end_time;
    int num_bytes_written;

    // Generate a test data packet to be serialized
    measurements.base_timestamp = esp_timer_get_time();
    measurements.dac_bias = 4095;
    measurements.excitation_voltage = 2.5;
    strcpy(measurements.mac_address, "AB:CD:EF:GH:IJ:KL");
    measurements.sample_channel_id = 0;

    for (int i = 0; i < NUM_SAMPLES_PER_MESSAGE; i++) {
        measurements.timestamp_deltas[i] = i * 500;
        measurements.values[i] = i * 3.000;
    }

    for (int i = 0; i < NUM_ENCODES; i++) {    
        start_time = esp_timer_get_time();
        num_bytes_written = hardEncoder::encodeDriveData(measurements, buffer_2);
        end_time = esp_timer_get_time();

        encode_times_micros[i] = end_time - start_time;
    }

    uint64_t total_micros = 0;
    for (int i = 0; i < NUM_ENCODES; i++) {
        total_micros += encode_times_micros[i];
    }

    for (int i = 0; i < 12000; i++) {
        if (buffer_1[i] != buffer_2[i]) {
            ESP_LOGI(TAG, "Index not equal: %d", i);
            break;
        }
    }

    ESP_LOGI(TAG, "Average hard encoding time (micros): %f", ((float) total_micros) / NUM_ENCODES);
    ESP_LOGI(TAG, "Hard encoded bytestream length (bytes): %u", num_bytes_written);

    pb_istream_t istream = pb_istream_from_buffer(buffer_2.data(), num_bytes_written);

    if (!pb_decode(&istream, wsg_drive_data_t_fields, &out_measurements)) {
        ESP_LOGI(TAG, "Failed to decode stream");
        return;
    }

    assert(out_measurements.base_timestamp == measurements.base_timestamp);
    assert(out_measurements.dac_bias == measurements.dac_bias);
    assert(out_measurements.excitation_voltage == measurements.excitation_voltage);
    assert(out_measurements.sample_channel_id == measurements.sample_channel_id);
    assert(strcmp(out_measurements.mac_address, measurements.mac_address) == 0);

    for (int i = 0; i < 10; i++) {
        assert(out_measurements.timestamp_deltas[i] == measurements.timestamp_deltas[i]);
        assert(out_measurements.values[i] == measurements.values[i]);
    }

    ESP_LOGI(TAG, "Finished testing Protobuf hard encoding");
}

#endif
