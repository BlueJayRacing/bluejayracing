#include "QoSWiFiMQTT.h"

#define MIN_SECURITY WIFI_AUTH_OPEN

bool QoSWiFiMQTT::wifiConnected = false;
bool QoSWiFiMQTT::mqttConnected = false;

QoSWiFiMQTT::QoSWiFiMQTT(char* wifi_ssid, char* wifi_pswd, int mqtt_port, uint8_t* mqtt_ip_addr, bool wifi_auto_reconnect = false) {
  this->wifi_ssid = wifi_ssid;
  this->wifi_pswd = wifi_pswd;
  this->mqtt_port = mqtt_port;
  this->wifi_auto_reconnect = wifi_auto_reconnect;
  this->mqtt_client = new espMqttClient();
  setUpMQTTCallbacks();
  WiFi.onEvent(wifiEvent);
  WiFi.setMinSecurity(MIN_SECURITY);

  IPAddress ip_address(10, 42, 0, 1);
  
  WiFi.setAutoReconnect(this->wifi_auto_reconnect);
  mqtt_client->setServer(ip_address, mqtt_port);
}

void QoSWiFiMQTT::connectToWiFi() {
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(this->wifi_ssid, this->wifi_pswd);  
}

void QoSWiFiMQTT::connectToMQTT() {
  Serial.print("Checking WiFi connection...");
  while (!wifiConnected) {
    delay(5000);
  }

  Serial.print("Connecting to MQTT...");
  mqtt_client->connect();
}

void QoSWiFiMQTT::publishMQTT(char* topic, char* message, int QoS) {
  while (!mqttConnected) {
    delay(100);
  }
  mqtt_client->publish(topic, QoS, false, message);
}

void QoSWiFiMQTT::subscribeMQTT(char* topic, int QoS) {
  mqtt_client->subscribe(topic, QoS);
}

void QoSWiFiMQTT::unsubscribeMQTT(char* topic) {
  mqtt_client->unsubscribe(topic);
}

bool QoSWiFiMQTT::isConnected() {
  return mqttConnected;
}

void QoSWiFiMQTT::setUpMQTTCallbacks() {
  mqtt_client->onConnect(onMQTTConnect);
  mqtt_client->onDisconnect(onMQTTDisconnect);
  mqtt_client->onSubscribe(onMQTTSubscribe);
  mqtt_client->onUnsubscribe(onMQTTUnsubscribe);
  mqtt_client->onMessage(onMQTTMessage);
  mqtt_client->onPublish(onMQTTPublish);
}

void QoSWiFiMQTT::wifiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    wifiConnected = false;
    break;
  default:
    break;
  }
}

void QoSWiFiMQTT::onMQTTMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total) {
  Serial.println("onMQTTMessage");
}

void QoSWiFiMQTT::onMQTTPublish(uint16_t packetId) {
  //Serial.println("onMQTTPublish");
}

void QoSWiFiMQTT::onMQTTUnsubscribe(uint16_t packetId) {
  Serial.println("onMQTTUnsubscribe");
}

void QoSWiFiMQTT::onMQTTSubscribe(uint16_t packetId, const espMqttClientTypes::SubscribeReturncode* codes, size_t len) {
  Serial.println("onMQTTSubscribe");
}

void QoSWiFiMQTT::onMQTTConnect(bool sessionPresent) {
  Serial.println("");
  Serial.println("Connected to MQTT");
  mqttConnected = true;
}

void QoSWiFiMQTT::onMQTTDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.println("");
  Serial.println("Disconnected from MQTT");
  mqttConnected = false;
}



