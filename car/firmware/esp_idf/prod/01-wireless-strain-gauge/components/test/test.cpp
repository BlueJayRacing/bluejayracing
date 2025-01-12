#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>
#include <esp_timer.h>

#define SPI2_MOSI_PIN 18
#define SPI2_MISO_PIN 20
#define SPI2_SCLK_PIN 19

#define ADC_VALUE_ERROR_MARGIN 0.001

static const char* TAG = "test";

Test::Test(esp_log_level_t test_log_level) { esp_log_level_set(TAG, test_log_level); }

void Test::testMemoryQueue(void)
{
    testMemoryQueueBasic();
    testMemoryQueueAcquireFull();
}

void Test::testMQTTManager(void)
{
    esp_log_level_set("wifi", ESP_LOG_NONE);
    esp_log_level_set("wifi_init", ESP_LOG_NONE);

    mqtt_manager_ = mqttManager::getInstance();

    // Initialize the MQTT Manager
    assert(mqtt_manager_->init() == ESP_OK);
    ESP_LOGD(TAG, "Initialized MQTT manager");

    // Run MQTT manager tests
    testMQTTManagerBasicParamErrors();
    testMQTTManagerWiFiConnectDisconnect();
    testMQTTManagerClientConnectDisconnect();
    testMQTTManagerClientWiFiConnectDisconnect();
    testMQTTManagerMultipleClientsConDisCon();
    testMQTTManagerClientPublishSubscribe();
}

/* We check basic memory queue functions like acquiring, writing to, and pushing
 * blocks onto the memory queue. We also cover function returns on invalid input
 * or invalid operation for that state.
 */
void Test::testMemoryQueueBasic(void)
{
    ESP_LOGI(TAG, "Testing Basic Memory Queue Functions");
    memoryQueue queue(10, 20);
    ESP_LOGD(TAG, "Initialized memory queue");

    // Try to correctly acquire a block to write on
    memoryBlock* acquired_block = queue.acquire();
    assert(acquired_block != nullptr);
    ESP_LOGD(TAG, "successfully acquired block");

    std::vector<uint8_t> data_vec;
    data_vec.reserve(20);
    for (int i = 0; i < 20; i++) {
        data_vec.push_back(i);
    }

    assert(acquired_block->write(data_vec) == 20);
    assert(acquired_block->write(data_vec) == 0);
    ESP_LOGD(TAG, "successfully written to block");

    // Try to acquire a block to write on while acquire_lock is on
    assert(queue.acquire() == nullptr);
    ESP_LOGD(TAG, "successfully prevented acquiring two blocks at once");

    // Try to correctly push a block that you have finished writing to
    assert(queue.push(acquired_block) == 0);
    assert(queue.getNumPushed() == 1);
    ESP_LOGD(TAG, "successfully pushed acquired block");

    // Try to correctly pop a block to read from
    memoryBlock popped_block(20);
    assert(queue.pop(popped_block) == 0);
    ESP_LOGD(TAG, "successfully popped block from queue");

    uint8_t* data = popped_block.data();
    for (int i = 0; i < 20; i++) {
        assert(*(data + i) == i);
    }
    ESP_LOGI(TAG, "Passed Basic Memory Queue Functions");
}

/* We check to see if the queue can properly free the oldest memory blocks when
 * acquiring memory blocks on a full queue.
 */
void Test::testMemoryQueueAcquireFull(void)
{
    ESP_LOGI(TAG, "Testing Memory Queue When Full");
    memoryQueue queue(10, 20);

    for (int i = 0; i < 20; i++) {
        std::vector<uint8_t> data_vec(20, i);

        memoryBlock* acquired_block = queue.acquire();
        assert(acquired_block != nullptr);
        assert(acquired_block->write(data_vec) == 20);
        queue.push(acquired_block);
        ESP_LOGD(TAG, "Pushed Block %d", i);
    }

    memoryBlock mem_block(20);
    uint8_t* data = mem_block.data();

    for (int i = 0; i < queue.size(); i++) {
        assert(queue.pop(mem_block) == 0);
        for (int j = 0; j < mem_block.size(); j++) {
            ESP_LOGD(TAG, "Data val: %d, Index: %d", *(data + j), i);
            assert(*(data + j) == (i + 10));
        }
    }
    ESP_LOGI(TAG, "Passed Memory Queue When Full");
}

/* We check to see if the MQTT manager returns the correct return values on error.
 */
void Test::testMQTTManagerBasicParamErrors(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager Basic Paramter Error Handling");

    mqtt_client_t client;

    assert(mqtt_manager_->connectWiFi("", "hi") == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->connectWiFi("hi", "") == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->connectWiFi(std::string('c', 33), "hi") == ESP_ERR_INVALID_SIZE);
    assert(mqtt_manager_->connectWiFi("hi", std::string('c', 65)) == ESP_ERR_INVALID_SIZE);

    assert(mqtt_manager_->createClient("") == NULL);

    assert(mqtt_manager_->clientConnect(NULL) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientConnect(&client) == ESP_ERR_WIFI_NOT_CONNECT); // Fail because WiFi not connected

    assert(mqtt_manager_->clientDisconnect(NULL) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->isClientConnected(NULL) == false);

    assert(mqtt_manager_->clientEnqueue(NULL, "payload", "topic", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientEnqueue(&client, "", "topic", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientEnqueue(&client, "payload", "", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientEnqueue(&client, "payload", "topic", 3) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientEnqueue(&client, "payload", "topic", 2) == ESP_ERR_WIFI_NOT_CONNECT);

    assert(mqtt_manager_->clientSubscribe(NULL, "topic", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientSubscribe(&client, "", 2) == ESP_ERR_INVALID_ARG);
    assert(mqtt_manager_->clientSubscribe(&client, "topic", 3) == ESP_ERR_INVALID_ARG);

    mqtt_message_t message;
    assert(mqtt_manager_->clientReceive(NULL, message) == ESP_ERR_INVALID_ARG);
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
        assert(mqtt_manager_->connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
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
        assert(mqtt_manager_->connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
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
        assert(mqtt_manager_->connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
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
        assert(mqtt_manager_->connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
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
        assert(mqtt_manager_->connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
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
        assert(mqtt_manager_->clientEnqueue(client, "hi " + std::to_string(i), "esp32/test_publish", 2) == ESP_OK);
        assert(mqtt_manager_->clientWaitPublish(client, 1000) == ESP_OK);
        assert(mqtt_manager_->clientWaitPublish(client, 10) == ESP_ERR_TIMEOUT);
        ESP_LOGD(TAG, "Client Published to MQTT");

        memset(&rec_mes, 0, sizeof(mqtt_message_t));

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_->clientReceive(client, rec_mes) == ESP_OK) {
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

void Test::testADCDAC(void)
{
    // Configure the SPI bus
    spi_bus_config_t spi_cfg;
    memset(&spi_cfg, 0, sizeof(spi_bus_config_t));

    spi_cfg.mosi_io_num   = SPI2_MOSI_PIN;
    spi_cfg.miso_io_num   = SPI2_MISO_PIN;
    spi_cfg.sclk_io_num   = SPI2_SCLK_PIN;
    spi_cfg.quadwp_io_num = -1;
    spi_cfg.quadhd_io_num = -1;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %d", ret);
        return;
    }

    // Initialize the DAC instance
    ad5626_init_param_t dac_params;
    dac_params.cs_pin   = GPIO_NUM_0;
    dac_params.ldac_pin = GPIO_NUM_23;
    dac_params.clr_pin  = GPIO_NUM_17;
    dac_params.spi_host = SPI2_HOST;

    ret = dac_.init(dac_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AD5626: %d", ret);
        return;
    }

    // Initialize the ADC instance
    ads1120_init_param_t adc_params;
    adc_params.cs_pin = GPIO_NUM_21;
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
    testADCDACReadAnalogFrontEnd();
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

    adc_regs.conv_mode = 1; // continuous
    adc_regs.op_mode = 2;   // turbo
    adc_regs.data_rate = 6; // 2000 SPS

    adc_.configure(adc_regs);

    uint16_t data;
    int num_success_reads = 0;
    int num_failed_reads = 0;
    int start_time = esp_timer_get_time();
    int current_time = esp_timer_get_time();

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

#define NUM_DAC_SAMPLES 315

/* We check whether the DAC bias is being set properly by having the ADC read it
 * as a single-ended input between the DAC bias and ground.
 */
void Test::testADCDACReadDACBias(void)
{
    ESP_LOGI(TAG, "Testing Reading DAC Bias from ADC");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = 1;
    adc_regs.op_mode = 2;
    adc_regs.analog_channels = 0X0A;
    adc_regs.data_rate = 3;
    adc_regs.volt_refs = 1;

    esp_err_t ret = adc_.configure(adc_regs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC: %d", ret);
        return;
    }

    float sum_diff = 0;

    for (int i = 0; i < NUM_DAC_SAMPLES; i++) {
        ret = dac_.setLevel(AD5626::MAX_LEVEL_VALUE * i / (NUM_DAC_SAMPLES));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set DAC level: %d", ret);
            continue;
        }

        vTaskDelay(5);

        while (!adc_.isDataReady()) {
            vTaskDelay(1);
        }
        
        uint16_t read_value;

        ret = adc_.readADC(&read_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ADC: %d", ret);
            continue;
        }

        float expected_dac_voltage = 4.095 * i / (NUM_DAC_SAMPLES);
        float measured_dac_voltage = 5.001 * read_value / ((1 << 15) - 1);
        sum_diff += std::abs(expected_dac_voltage - measured_dac_voltage);

        ESP_LOGD(TAG, "Expected DAC Voltage Value: %.10f", expected_dac_voltage);
        ESP_LOGD(TAG, "Measured DAC Voltage Value: %.10f", measured_dac_voltage);

        assert((measured_dac_voltage - ADC_VALUE_ERROR_MARGIN < expected_dac_voltage) && (measured_dac_voltage + ADC_VALUE_ERROR_MARGIN > expected_dac_voltage));
    }

    ESP_LOGI(TAG, "Average error (V): %f", sum_diff / NUM_DAC_SAMPLES);

    ESP_LOGI(TAG, "Passed reading DAC Bias from ADC");
}

void Test::testADCDACReadAnalogFrontEnd(void)
{
    ESP_LOGI(TAG, "Testing Reading Analog FrontEnd");

    ads1120_regs_t adc_regs;
    memset(&adc_regs, 0, sizeof(ads1120_regs_t));

    adc_regs.conv_mode = 1;
    adc_regs.op_mode = 2;
    adc_regs.analog_channels = 0X0A;
    adc_regs.data_rate = 3;
    adc_regs.volt_refs = 1;
}