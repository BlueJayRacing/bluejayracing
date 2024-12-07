#pragma once
#ifndef _MQTT_MANAGER_HPP_
#define _MQTT_MANAGER_HPP_

#include <array>
#include <esp_event.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <mqtt_client.h>
#include <string>
#include <vector>

typedef struct mqtt_client {
  esp_mqtt_client_handle_t client_;
  QueueHandle_t rec_queue_;
  EventGroupHandle_t conn_event_;

  mqtt_client(void) : client_(NULL), rec_queue_(NULL), conn_event_(NULL) {};
} mqtt_client_t;

typedef struct mqtt_message {
    std::array<char, 20> topic;
    std::array<uint8_t, 30> payload;
} mqtt_message_t;

/*******************************************************************************
 * @brief An MQTT Manager that manages a single MQTT connection between the ESP32XX
 *        and a broker through WiFi. Only one MQTT Manager instance should be declared
 *        per application.
 *******************************************************************************/
class mqttManager {
  public:
    static mqttManager* getInstance();
    esp_err_t init(void);
    esp_err_t connectWiFi(const std::string& wifi_ssid, const std::string& wifi_pswd);
    void disconnectWiFi(void);
    bool isWiFiConnected(void) const;

    mqtt_client_t* createClient(const std::string& t_broker_uri);
    void destroyClient(mqtt_client_t* mqtt_client);
    esp_err_t connectClient(mqtt_client_t* mqtt_client);
    esp_err_t disconnectClient(mqtt_client_t* mqtt_client);
    bool isClientConnected(mqtt_client_t* mqtt_client) const;
    esp_err_t enqueueClient(mqtt_client_t* mqtt_client, const std::vector<char>& payload, const std::vector<char>& topic, uint8_t QoS);
    esp_err_t waitPublishClient(mqtt_client_t* mqtt_client, TickType_t timeout);
    esp_err_t subscribeClient(mqtt_client_t* mqtt_client, const std::vector<char>& topic, uint8_t QoS);
    esp_err_t receiveClient(mqtt_client_t* mqtt_client, mqtt_message_t& message);
    esp_err_t clearClientMessages(mqtt_client_t* mqtt_client);

  private:
    mqttManager(void);
    ~mqttManager();
    static int client_id_counter;
    static void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    static void mqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);

  private:
    esp_netif_t* wifi_netif_;
    static SemaphoreHandle_t mutex_;
    static mqttManager* instance_;
    static EventGroupHandle_t wifi_conn_group_;
};

#endif