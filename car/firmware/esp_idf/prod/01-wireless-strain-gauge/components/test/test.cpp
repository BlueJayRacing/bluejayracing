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

    mqtt_manager_ = mqttManager::getInstance();

    assert(mqtt_manager_->init() == ESP_OK);
    ESP_LOGD(TAG, "Initialized MQTT manager");

    testMQTTManagerBasicParamErrors();
    testMQTTManagerWiFiConnectDisconnect();
    testMQTTManagerClientConnectDisconnect();
    testMQTTManagerClientWiFiConnectDisconnect();
    testMQTTManagerMultipleClientsConDisCon();
    testMQTTManagerClientPublishSubscribe();
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