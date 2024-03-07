#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

class WiFiMQTT {
    public:
        WiFiMQTT(char* wifi_ssid, char* wifi_pswd, char* mqtt_server_ip, int mqtt_server_port);
        void beginWifi();
        void beginMQTT();
        void sendMQTTMessage(char* topic, uint8_t* message, unsigned int length);
        void reconnectMQTT();
        char* getWifiSSID();
        char* getWifiPSWD();
        char* getMQTTServerIP();
        int getMQTTPort();
        bool isConnected();
    private:
        char* wifi_ssid;
        char* wifi_pswd;
        char* mqtt_server_ip;
        int mqtt_server_port;
        PubSubClient* client;
        WiFiClient espClient;
        static void callback(char* topic, byte* message, unsigned int length);
};