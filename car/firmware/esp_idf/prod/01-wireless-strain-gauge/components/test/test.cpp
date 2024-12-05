#include <esp_log.h>
#include <esp_system.h>
#include <test.hpp>

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

    assert(mqtt_manager_.init() == ESP_OK);
    ESP_LOGD(TAG, "Initialized MQTT manager");

    testMQTTManagerWiFiConnectDisconnect();
    testMQTTManagerMQTTConnectDisconnect();
    // testMQTTManagerMQTTWiFiConnectDisconnect();
    // testMQTTManagerMQTTPublishSubscribe();
}

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

void Test::testMQTTManagerWiFiConnectDisconnect(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager WiFi Connection");

    for (int i = 0; i < 10; i++) {
        assert(mqtt_manager_.connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");

        mqtt_manager_.stopWiFi();

        for (int j = 0; j < 5; j++) {
            if (!mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
        vTaskDelay(200);
    }

    ESP_LOGI(TAG, "Passed MQTT Manager WiFi Connection");
}

void Test::testMQTTManagerMQTTConnectDisconnect(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager MQTT Connection");

    // Initialization (Connects to WiFi)
    {
        assert(mqtt_manager_.connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");
    }

    for (int i = 0; i < 10; i++) {
        assert(mqtt_manager_.connectMQTT("mqtt://10.42.0.1") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to MQTT");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_.isMQTTConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isMQTTConnected() == true);
        ESP_LOGD(TAG, "Connected to MQTT");

        mqtt_manager_.stopMQTT();

        for (int j = 0; j < 5; j++) {
            if (!mqtt_manager_.isMQTTConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isMQTTConnected() == false);
        ESP_LOGD(TAG, "Disconnected from MQTT");
        vTaskDelay(200);
    }

    // Deinitialization (Discnnects from WiFi)
    {
        mqtt_manager_.stopWiFi();

        for (int i = 0; i < 5; i++) {
            if (!mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
    }

    ESP_LOGI(TAG, "Passed MQTT Manager MQTT Connection");
}

void Test::testMQTTManagerMQTTWiFiConnectDisconnect(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager MQTT + WIFI Disconnect");

    // Initialization (Connects to WiFi + MQTT)
    {
        assert(mqtt_manager_.connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");

        assert(mqtt_manager_.connectMQTT("mqtt://10.42.0.1") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to MQTT");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_.isMQTTConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isMQTTConnected() == true);
        ESP_LOGD(TAG, "Connected to MQTT");
    }

    mqtt_manager_.stopWiFi();

    for (int j = 0; j < 5; j++) {
        if (!mqtt_manager_.isWiFiConnected()) {
            break;
        }
        vTaskDelay(100);
    }

    assert(mqtt_manager_.isWiFiConnected() == false);
    assert(mqtt_manager_.isMQTTConnected() == false);
    ESP_LOGD(TAG, "Disconnected from WiFi and MQTT");

    ESP_LOGI(TAG, "Passed MQTT Manager MQTT + WIFI Disconnect");
}

void Test::testMQTTManagerMQTTPublishSubscribe(void)
{
    ESP_LOGI(TAG, "Testing MQTT Manager MQTT Publish");

    // Initialization (Connects to WiFi + MQTT)
    {
        assert(mqtt_manager_.connectWiFi("bjr_wireless_axle_host", "bluejayracing") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to WiFi");

        for (int i = 0; i < 30; i++) {
            if (mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == true);
        ESP_LOGD(TAG, "Connected to WiFi");

        assert(mqtt_manager_.connectMQTT("mqtt://10.42.0.1") == ESP_OK);
        ESP_LOGD(TAG, "Started Connecting to MQTT");

        for (int j = 0; j < 30; j++) {
            if (mqtt_manager_.isMQTTConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isMQTTConnected() == true);
        ESP_LOGD(TAG, "Connected to MQTT");
    }

    // Deinitialization (Disconnects from WiFi + MQTT)
    {
        mqtt_manager_.stopMQTT();

        for (int j = 0; j < 5; j++) {
            if (!mqtt_manager_.isMQTTConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isMQTTConnected() == false);
        ESP_LOGD(TAG, "Disconnected from MQTT");

        mqtt_manager_.stopWiFi();

        for (int i = 0; i < 5; i++) {
            if (!mqtt_manager_.isWiFiConnected()) {
                break;
            }
            vTaskDelay(100);
        }

        assert(mqtt_manager_.isWiFiConnected() == false);
        ESP_LOGD(TAG, "Disconnected from WiFi");
    }

    ESP_LOGI(TAG, "Passed MQTT Manager MQTT Publish (have to validate by looking at RPi5)");
}