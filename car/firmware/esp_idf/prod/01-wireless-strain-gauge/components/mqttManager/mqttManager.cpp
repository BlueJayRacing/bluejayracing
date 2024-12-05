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
#include <mqtt_client.h>
#include <nvs_flash.h>

#define MQTT_CONNECTED_BIT BIT1
#define WIFI_CONNECTED_BIT BIT0

static const char* TAG = "mqttManager";

mqttManager* mqttManager::instance_ = NULL;
QueueHandle_t mqttManager::rec_message_queue_     = xQueueCreate(5, sizeof(mqtt_message_t));
EventGroupHandle_t mqttManager::conn_event_group_ = xEventGroupCreate();

mqttManager* mqttManager::getInstance() {
    if (!instance_) {
        instance_ = new mqttManager();
    }
    return instance_;
}

mqttManager::mqttManager() : wifi_netif_(NULL), mqtt_handle_(NULL) {}

mqttManager::~mqttManager()
{
}

/*******************************************************************************
 * @brief Initializes the MQTT Manager.
 *
 * @return Returns 0 for success or negative error code.
 *
 * @note Should only be called once per application.
 *******************************************************************************/
esp_err_t mqttManager::init(void)
{
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
    err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11N);
    if (err) {
        ESP_LOGE(TAG, "Failed to set WiFi Mode (err: %d)\n", err);
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

/*******************************************************************************
 * @brief Connects to the MQTT Broker
 *
 * @param t_broker_uri The URI of the MQTT Broker
 * 
 * @return Returns 0 for success or negative error code.
 * 
 * @note Will not attempt to connect to MQTT unless WiFi is also connected
 *******************************************************************************/
esp_err_t mqttManager::connectMQTT(const std::string& t_broker_uri)
{
    if (t_broker_uri.length() == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (!isWiFiConnected()) {
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    if (mqtt_handle_ != NULL) {
        esp_mqtt_client_destroy(mqtt_handle_);
    }

    esp_mqtt_client_config_t mqtt_config;
    memset(&mqtt_config, 0, sizeof(esp_mqtt_client_config_t));

    mqtt_config.broker.address.uri = t_broker_uri.data();

    mqtt_handle_ = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_handle_ == NULL) {
        return ESP_ERR_NOT_FINISHED;
    }

    esp_err_t err = esp_mqtt_client_register_event(mqtt_handle_, MQTT_EVENT_ANY, mqttEventHandler, NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_mqtt_client_start(mqtt_handle_);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

/*******************************************************************************
 * @brief Disconnects from WiFi
 *
 * @note Will also trigger a disconnect for MQTT
 *******************************************************************************/
void mqttManager::disconnectWiFi(void) { esp_wifi_disconnect(); }

/*******************************************************************************
 * @brief Disconnects from MQTT (will not impact WiFi connection)
 *******************************************************************************/
void mqttManager::disconnectMQTT(void)
{
    if (mqtt_handle_ != NULL) {
        esp_mqtt_client_disconnect(mqtt_handle_);
    }
}

void mqttManager::wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch ((wifi_event_t)event_id) {
        case WIFI_EVENT_STA_START:
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(conn_event_group_, WIFI_CONNECTED_BIT);
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        switch ((ip_event_t)event_id) {
        case IP_EVENT_STA_GOT_IP:
            xEventGroupSetBits(conn_event_group_, WIFI_CONNECTED_BIT);
            break;
        default:
            break;
        }
    }
}

void mqttManager::mqttEventHandler(void* args, esp_event_base_t base, int32_t event_id, void* event_data)
{
    BaseType_t err = pdFALSE;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_t*)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        xEventGroupSetBits(conn_event_group_, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        xEventGroupClearBits(conn_event_group_, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        break;
    case MQTT_EVENT_PUBLISHED:
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
        ESP_LOGI(TAG, "DATA=%.*s\r\n", event->data_len, event->data);

        mqtt_message_t message;

        if (uxQueueSpacesAvailable(rec_message_queue_) == 0) {
            err = xQueueReceive(rec_message_queue_, &message, 5);
            if (!err) {
                return;
            }
        }

        memcpy(message.topic.data(), event->topic, std::min(event->topic_len, (int)message.topic.size()));
        memcpy(message.payload.data(), event->data, std::min(event->data_len, (int)message.payload.size()));
        xQueueSend(rec_message_queue_, &message, 5);
        break;
    case MQTT_EVENT_ERROR:
        break;
    default:
        break;
    }
}

bool mqttManager::isWiFiConnected(void) const { return xEventGroupGetBits(conn_event_group_) & WIFI_CONNECTED_BIT; }

bool mqttManager::isMQTTConnected(void) const { return xEventGroupGetBits(conn_event_group_) & MQTT_CONNECTED_BIT; }

esp_err_t mqttManager::publishMQTT(const std::vector<char>& t_payload, const std::vector<char>& t_topic, uint8_t t_QoS)
{
    if (!isWiFiConnected() || !isMQTTConnected()) {
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    return esp_mqtt_client_publish(mqtt_handle_, t_topic.data(), t_payload.data(), t_payload.size(), 0, 0);
}

esp_err_t mqttManager::subscribeMQTT(const std::vector<char>& t_topic, uint8_t t_QoS)
{
    if (!isWiFiConnected() || !isMQTTConnected()) {
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    return esp_mqtt_client_subscribe(mqtt_handle_, t_topic.data(), t_QoS);
}

esp_err_t mqttManager::receiveMQTT(mqtt_message_t& t_payload)
{
    if (uxQueueSpacesAvailable(rec_message_queue_) == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    if (xQueueReceive(rec_message_queue_, &t_payload, 1) != pdTRUE) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void mqttManager::clearMQTTMessages(void) { xQueueReset(rec_message_queue_); }