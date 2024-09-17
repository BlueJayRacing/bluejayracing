#ifndef QOS_MQTT_H
#define QOS_MQTT_H

#include <WiFi.h>
#include <espMqttClient.h>

class qos_mqtt {
    public:
        qos_mqtt(int mqtt_port, uint8_t* mqtt_ip_addr, bool wifi_auto_reconnect);
        void connect_mqtt();
        void publish_mqtt(char* topic, uint8_t* message, int length, int QoS);
        void subscribe_mqtt(char* topic, int QoS);
        void unsubscribe_mqtt(char* topic);
        bool is_connected();
    private:
        void set_up_mqtt_callbacks();
        static void on_mqtt_connect(bool sessionPresent);
        static void on_mqtt_message(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total);
        static void on_mqtt_publish(uint16_t packetId);
        static void on_mqtt_unsubscribe(uint16_t packetId);
        static void on_mqtt_subscribe(uint16_t packetId, const espMqttClientTypes::SubscribeReturncode* codes, size_t len);
        static void on_mqtt_disconnect(espMqttClientTypes::DisconnectReason reason);
        espMqttClient* mqtt_client;
        IPAddress* mqtt_ip_addr;
        int mqtt_port;
        bool wifi_auto_reconnect;
        static bool mqtt_connected;
};

#endif