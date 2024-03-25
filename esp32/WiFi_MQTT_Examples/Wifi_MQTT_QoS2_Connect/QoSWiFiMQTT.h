#include <WiFi.h>
#include <espMqttClient.h>

#ifndef QOS_WIFI_MQTT_H
#define QOS_WIFI_MQTT_H

class QoSWiFiMQTT {
    public:
        QoSWiFiMQTT(char* wifi_ssid, char* wifi_pswd, int mqtt_port, uint8_t* mqtt_ip_addr, bool wifi_auto_reconnect);
        void connectToWiFi();
        void connectToMQTT();
        void publishMQTT(char* topic, uint8_t* message, int length, int QoS);
        void subscribeMQTT(char* topic, int QoS);
        void unsubscribeMQTT(char* topic);
        bool isConnected();
    private:
        void setUpMQTTCallbacks();
        static void wifiEvent(WiFiEvent_t event);
        static void onMQTTConnect(bool sessionPresent);
        static void onMQTTMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total);
        static void onMQTTPublish(uint16_t packetId);
        static void onMQTTUnsubscribe(uint16_t packetId);
        static void onMQTTSubscribe(uint16_t packetId, const espMqttClientTypes::SubscribeReturncode* codes, size_t len);
        static void onMQTTDisconnect(espMqttClientTypes::DisconnectReason reason);
        espMqttClient* mqtt_client;
        IPAddress* mqtt_ip_addr;
        char* wifi_ssid;
        char* wifi_pswd;
        int mqtt_port;
        bool wifi_auto_reconnect;
        static bool wifiConnected;
        static bool mqttConnected;
        static int numPub;
};

#endif