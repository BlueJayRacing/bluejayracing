#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#ifndef WIFIMQTT_H
#define WIFIMQTT_H

class WiFiMQTT {
public:
  WiFiMQTT(char* wifi_ssid, char* wifi_pswd, char* mqtt_server_ip, int mqtt_server_port);
  void beginWifi();
  void beginMQTT();
  void sendMQTTMessage(char* topic, uint8_t* message, unsigned int length);
  char* getWifiSSID();
  char* getWifiPSWD();
  char* getMQTTServerIP();
  int getMQTTPort();
  bool isConnected();
private:
  void reconnect();
  void reconnectMQTT();
  void reconnectWiFi();
  char* wifi_ssid;
  char* wifi_pswd;
  char* mqtt_server_ip;
  int mqtt_server_port;
  PubSubClient* client;
  WiFiClient esp_client;
  static void callback(char* topic, byte* message, unsigned int length);
};

#endif