#include "WifiMQTT.h"

WiFiMQTT::WiFiMQTT(char* wifi_ssid, char* wifi_pswd, char* mqtt_server_ip, int mqtt_server_port) {
    this->wifi_ssid = wifi_ssid;
    this->wifi_pswd = wifi_pswd;
    this->mqtt_server_ip = mqtt_server_ip;
    this->mqtt_server_port = mqtt_server_port;
    this->client = new PubSubClient(espClient);
}

void WiFiMQTT::beginWifi() {

    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(this->wifi_ssid);

    WiFi.setMinSecurity(WIFI_AUTH_OPEN);
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

    client->connect(WiFi.macAddress().c_str());
    while (!client->connected()) {
      Serial.print(".");
      delay(1);
    }
    Serial.println();
}

void WiFiMQTT::callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println();
}

void WiFiMQTT::sendMQTTMessage(char* topic, uint8_t* message, unsigned int length) {
    client->publish(topic, message, length);
}

void WiFiMQTT::reconnectMQTT() {
  Serial.print("Attempting MQTT connection...");
  
  if (client->connect(WiFi.macAddress().c_str())) {
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