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
    esp_mqtt_client_handle_t client_handle;
    QueueHandle_t rec_queue;
    EventGroupHandle_t conn_event;

    mqtt_client(void) : client_handle(NULL), rec_queue(NULL), conn_event(NULL){};
} mqtt_client_t;

typedef struct mqtt_message {
    std::array<char, 20> topic;
    std::array<uint8_t, 30> payload;
    uint8_t payload_len;
<<<<<<<< HEAD:car/firmware/esp_idf/components/mqtt_manager/include/mqttManager.hpp
    uint32_t esp_time;
========
>>>>>>>> main:car/firmware/esp_idf/components/mqttManager/include/mqttManager.hpp
} mqtt_message_t;

/*******************************************************************************
 * @brief An MQTT Manager that manages a single MQTT connection between the ESP32XX
 *        and a broker through WiFi. Only one MQTT Manager instance should be declared
 *        per application.
 *******************************************************************************/
class mqttManager {
  public:
    static mqttManager* getInstance();
    esp_err_t init(const std::string& t_ssid, const std::string& t_pswd);
    esp_err_t connectWiFi(void);
    esp_err_t waitWiFiConnect(TickType_t ticks_to_wait);
    void disconnectWiFi(void);
    bool isWiFiConnected(void) const;

    mqtt_client_t* createClient(const std::string& t_broker_uri);
    void destroyClient(mqtt_client_t* client);
    esp_err_t clientConnect(mqtt_client_t* client);
    esp_err_t clientDisconnect(mqtt_client_t* client);
    bool isClientConnected(mqtt_client_t* client) const;
<<<<<<<< HEAD:car/firmware/esp_idf/components/mqtt_manager/include/mqttManager.hpp
    esp_err_t clientPublish(mqtt_client_t* client, uint8_t* buf, uint16_t buf_length, const std::string& topic, uint8_t QoS);
    // esp_err_t clientWaitPublish(mqtt_client_t* client, TickType_t timeout);
========
    esp_err_t clientEnqueue(mqtt_client_t* client, uint8_t* buf, uint16_t buf_length, const std::string& topic, uint8_t QoS);
    esp_err_t clientWaitPublish(mqtt_client_t* client, TickType_t timeout);
>>>>>>>> main:car/firmware/esp_idf/components/mqttManager/include/mqttManager.hpp
    esp_err_t clientSubscribe(mqtt_client_t* client, const std::string& topic, uint8_t QoS);
    esp_err_t clientWaitSubscribe(mqtt_client_t* client, TickType_t timeout);
    esp_err_t clientReceive(mqtt_client_t* client, mqtt_message_t& message, TickType_t timeout);
    esp_err_t clientClearMessages(mqtt_client_t* client);

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