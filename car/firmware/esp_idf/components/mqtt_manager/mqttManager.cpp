#include <mqttManager.hpp>

#include <algorithm>
#include <assert.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <lockGuard.hpp>
#include <mqtt_client.h>
#include <nvs_flash.h>

#define MQTT_CONNECTED_BIT  BIT0
#define MQTT_PUBLISHED_BIT  BIT1
#define MQTT_SUBSCRIBED_BIT BIT1

#define WIFI_CONNECTED_BIT BIT0

static const char* TAG = "mqttManager";

mqttManager* mqttManager::instance_              = NULL;
SemaphoreHandle_t mqttManager::mutex_            = xSemaphoreCreateMutex();
EventGroupHandle_t mqttManager::wifi_conn_group_ = xEventGroupCreate();
int mqttManager::client_id_counter               = 0;

mqttManager* mqttManager::getInstance()
{
    if (!instance_) {
        instance_ = new mqttManager();
    }
    return instance_;
}

mqttManager::mqttManager() : wifi_netif_(NULL) {}

mqttManager::~mqttManager() {}

/*******************************************************************************
 * @brief Initializes the MQTT Manager.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note Should only be called once per application.
 *******************************************************************************/
esp_err_t mqttManager::init(void)
{
    lockGuard guard(mutex_);
    esp_err_t err;

    err = nvs_flash_init();
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize NVS Flash (err: %d)", err);
        return err;
    }
    err = esp_netif_init();
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize Network Interface (err: %d)", err);
        return err;
    }
    err = esp_event_loop_create_default();
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize Event Loop (err: %d)", err);
        return err;
    }

    wifi_netif_ = esp_netif_create_default_wifi_sta();
    if (wifi_netif_ == NULL) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
        return ESP_FAIL;
    }

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventHandler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifiEventHandler, NULL);

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();

    err = esp_wifi_init(&wifi_init_config);
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize WiFi (err: %d)\n", err);
        return err;
    }
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err) {
        ESP_LOGE(TAG, "Failed to set WiFi Mode (err: %d)\n", err);
        return err;
    }

    return ESP_OK;
}

/*******************************************************************************
 * @brief Connects to the WiFi Access Point
 *
 * @param t_ssid The SSID of the WiFi Access Point
 * @param t_pwd  The password of the WiFi Access Point
 *
 * @return Returns 0 for success or negative error code.
 *******************************************************************************/
esp_err_t mqttManager::connectWiFi(const std::string& t_ssid, const std::string& t_pswd)
{
    lockGuard guard(mutex_);

    if (t_ssid.length() == 0 || t_pswd.length() == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    wifi_config_t wifi_config;

    if (t_ssid.length() > sizeof(wifi_config.sta.ssid) || t_pswd.length() > sizeof(wifi_config.sta.password)) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(&wifi_config, 0, sizeof(wifi_config_t));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    wifi_config.sta.scan_method        = WIFI_FAST_SCAN;
    wifi_config.sta.listen_interval    = 3;

    memcpy(&wifi_config.sta.ssid, t_ssid.data(), t_ssid.length());
    memcpy(&wifi_config.sta.password, t_pswd.data(), t_pswd.length());

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err) {
        ESP_LOGE(TAG, "Failed to set WiFi configuration (err: %d)\n", err);
        return err;
    }

    err = esp_wifi_start();
    if (err) {
        ESP_LOGE(TAG, "Failed to start WiFi (err: %d)\n", err);
        return err;
    }

    err = esp_wifi_connect();
    if (err) {
        ESP_LOGE(TAG, "Failed to start connecting to WiFi (err: %d)\n", err);
        return err;
    }

    return ESP_OK;
}

mqtt_client_t* mqttManager::createClient(const std::string& t_broker_uri)
{
    lockGuard guard(mutex_);

    if (t_broker_uri.length() == 0) {
        return NULL;
    }

    esp_mqtt_client_config_t mqtt_config;
    memset(&mqtt_config, 0, sizeof(esp_mqtt_client_config_t));

    std::string client_id(std::to_string(client_id_counter++));

    mqtt_config.broker.address.uri    = t_broker_uri.data();
    mqtt_config.credentials.client_id = client_id.data();

    mqtt_client_t* client = new mqtt_client_t();

    client->client_handle = esp_mqtt_client_init(&mqtt_config);
    if (client->client_handle == NULL) {
        delete client;
        return NULL;
    }

    esp_err_t err = esp_mqtt_client_register_event(client->client_handle, MQTT_EVENT_ANY, mqttEventHandler, client);
    if (err != ESP_OK) {
        esp_mqtt_client_destroy(client->client_handle);
        delete client;
        return NULL;
    }

    client->rec_queue  = xQueueCreate(5, sizeof(mqtt_message_t));
    client->conn_event = xEventGroupCreate();

    return client;
}

void mqttManager::destroyClient(mqtt_client_t* client)
{
    if (client == NULL) {
        return;
    }
    esp_mqtt_client_destroy(client->client_handle);
    vQueueDelete(client->rec_queue);
    vEventGroupDelete(client->conn_event);
    delete client;
}

esp_err_t mqttManager::clientConnect(mqtt_client_t* client)
{
    if (client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!isWiFiConnected()) {
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    esp_err_t err = esp_mqtt_client_start(client->client_handle);
    if (err == ESP_FAIL) {
        return esp_mqtt_client_reconnect(client->client_handle);
    } else {
        return err;
    }

    return ESP_OK;
}

void mqttManager::disconnectWiFi(void) { esp_wifi_disconnect(); }

esp_err_t mqttManager::clientDisconnect(mqtt_client_t* client)
{
    if (client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_mqtt_client_disconnect(client->client_handle);
    return ESP_OK;
}

void mqttManager::wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch ((wifi_event_t)event_id) {
        case WIFI_EVENT_STA_START:
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(wifi_conn_group_, WIFI_CONNECTED_BIT);
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        switch ((ip_event_t)event_id) {
        case IP_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_conn_group_, WIFI_CONNECTED_BIT);
            break;
        default:
            break;
        }
    }
}

void mqttManager::mqttEventHandler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    mqtt_client_t* client = (mqtt_client_t*)arg;
    BaseType_t err        = pdFALSE;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_t*)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        xEventGroupSetBits(client->conn_event, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        xEventGroupClearBits(client->conn_event, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        xEventGroupSetBits(client->conn_event, MQTT_SUBSCRIBED_BIT);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "Published Message");
        xEventGroupSetBits(client->conn_event, MQTT_PUBLISHED_BIT);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Topic (length: %d) = %.*s\r\n", event->topic_len, event->topic_len, event->topic);
        ESP_LOGI(TAG, "Data (length: %d) = %.*s\r\n", event->data_len, event->data_len, event->data);

        mqtt_message_t message;
        memset(&message, 0, sizeof(mqtt_message_t));

        if (uxQueueSpacesAvailable(client->rec_queue) == 0) {
            err = xQueueReceive(client->rec_queue, &message, 5);
            if (!err) {
                return;
            }
        }

        memcpy(message.topic.data(), event->topic, std::min(event->topic_len, (int)message.topic.size()));
        memcpy(message.payload.data(), event->data, std::min(event->data_len, (int)message.payload.size()));
        message.payload_len = event->data_len;
        xQueueSend(client->rec_queue, &message, 5);
        break;
    case MQTT_EVENT_ERROR:
        break;
    default:
        break;
    }
}

bool mqttManager::isWiFiConnected(void) const { return xEventGroupGetBits(wifi_conn_group_) & WIFI_CONNECTED_BIT; }

bool mqttManager::isClientConnected(mqtt_client_t* client) const
{
    if (client == NULL) {
        return false;
    } else {
        return xEventGroupGetBits(client->conn_event) & MQTT_CONNECTED_BIT;
    }
}

esp_err_t mqttManager::clientEnqueue(mqtt_client_t* client, uint8_t* buf, uint16_t buf_length, const std::string& t_topic,
                                     uint8_t t_QoS)
{
    if (client == NULL || t_topic.size() == 0 || buf_length == 0 || t_QoS >= 3) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!isWiFiConnected() || !isClientConnected(client)) {
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    int msg_id = esp_mqtt_client_enqueue(client->client_handle, t_topic.data(), (char*) buf, buf_length,
                                         t_QoS, true, true);
    if (msg_id == -1 || msg_id == -2) {
        ESP_LOGE(TAG, "Failed to publish for client (err: %d)\n", msg_id);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mqttManager::clientWaitPublish(mqtt_client_t* client, TickType_t timeout)
{
    EventBits_t ret = xEventGroupWaitBits(client->conn_event, MQTT_PUBLISHED_BIT, true, true, timeout);
    if ((ret & MQTT_PUBLISHED_BIT) != 0) {
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t mqttManager::clientSubscribe(mqtt_client_t* client, const std::string& t_topic, uint8_t t_QoS)
{
    if (client == NULL || t_topic.length() == 0 || t_QoS >= 3) {
        ESP_LOGE(TAG, "subscribeClient: invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (!isWiFiConnected() || !isClientConnected(client)) {
        ESP_LOGE(TAG, "subsctibeClient: MQTT not connected");
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    int msg_id = esp_mqtt_client_subscribe(client->client_handle, t_topic.data(), t_QoS);
    if (msg_id == -1 || msg_id == -2) {
        ESP_LOGE(TAG, "Failed to subscribe for client (err: %d)\n", msg_id);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mqttManager::clientWaitSubscribe(mqtt_client_t* client, TickType_t timeout)
{
    EventBits_t ret = xEventGroupWaitBits(client->conn_event, MQTT_SUBSCRIBED_BIT, true, true, timeout);
    if ((ret & MQTT_SUBSCRIBED_BIT) != 0) {
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

esp_err_t mqttManager::clientReceive(mqtt_client_t* client, mqtt_message_t& t_payload, TickType_t timeout)
{
    if (client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (uxQueueSpacesAvailable(client->rec_queue) == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueReceive(client->rec_queue, &t_payload, timeout) != pdTRUE) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mqttManager::clientClearMessages(mqtt_client_t* client)
{
    if (client == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xQueueReset(client->rec_queue);
    return ESP_OK;
}