#include "WifiMQTT.h"

#define MIN_SECURITY WIFI_AUTH_OPEN

WiFiMQTT::WiFiMQTT(char* wifi_ssid, char* wifi_pswd, char* mqtt_server_ip, int mqtt_server_port) {
    this->wifi_ssid = wifi_ssid;
    this->wifi_pswd = wifi_pswd;
    this->mqtt_server_ip = mqtt_server_ip;
    this->mqtt_server_port = mqtt_server_port;
    this->client = new PubSubClient(esp_client);
}

void WiFiMQTT::beginWifi() {

    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(this->wifi_ssid);

    WiFi.setMinSecurity(MIN_SECURITY);
    WiFi.begin(this->wifi_ssid, this->wifi_pswd);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void WiFiMQTT::beginMQTT() {
    client->setServer(mqtt_server_ip, mqtt_server_port);
    client->setCallback(callback);
    client->connect("ESP8266Client");
    while (!client->connected()) {
      Serial.println("MQTT still connecting...");
      delay(4);
    }
}

void WiFiMQTT::callback(char* topic, byte* message, unsigned int message_length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < message_length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println();
}

void WiFiMQTT::sendMQTTMessage(char* topic, uint8_t* message, unsigned int message_length) {
    client->publish(topic, message, message_length);
}

void WiFiMQTT::reconnectMQTT() {
  Serial.print("Attempting MQTT connection...");
    
  if (client->connect("ESP8266Client")) {
    Serial.println("connected");
    client->subscribe("esp32/output");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client->state());
    Serial.println(" try again in 5 seconds");
    delay(5000);
  }
}

char* WiFiMQTT::getWifiSSID() {
    return this->wifi_ssid;
}

char* WiFiMQTT::getWifiPSWD() {
    return this->wifi_pswd;
}

char* WiFiMQTT::getMQTTServerIP() {
    return this->mqtt_server_ip;
}

int WiFiMQTT::getMQTTPort() {
    return this->mqtt_server_port;
}

bool WiFiMQTT::isConnected() {
  return client->connected();
}