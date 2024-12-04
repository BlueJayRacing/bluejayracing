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

typedef struct mqtt_message {
    std::array<char, 20> topic;
    std::array<uint8_t, 30> payload;
} mqtt_message_t;

class mqttManager {
  public:
    mqttManager(void);
    ~mqttManager();
    esp_err_t init(void); //
    esp_err_t startWiFi(const std::string& wifi_ssid, const std::string& wifi_pswd); //
    esp_err_t connectWiFi(void); //
    esp_err_t connectMQTT(const std::string& broker_uri); //
    void stopWiFi(void); //
    void stopMQTT(void); //
    bool isWiFiConnected(void) const;
    bool isMQTTConnected(void) const;
    esp_err_t publishMQTT(const std::vector<char>& payload, const std::vector<char>& topic, uint8_t QoS);
    esp_err_t subscribeMQTT(const std::vector<char>& topic, uint8_t QoS);
    esp_err_t receiveMQTT(mqtt_message_t& message);

  private:
    static void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data); //
    static void mqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data); //

  private:
    esp_netif_t* wifi_netif_;
    esp_mqtt_client_handle_t mqtt_handle_;
    static QueueHandle_t rec_message_queue_;
    static EventGroupHandle_t conn_event_group_;
};

#endif