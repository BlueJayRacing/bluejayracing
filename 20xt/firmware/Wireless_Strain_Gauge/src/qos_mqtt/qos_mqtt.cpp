#include "qos_mqtt.h"

bool qos_mqtt::mqtt_connected = false;
char qos_mqtt::last_message[25] = "nosend";

qos_mqtt::qos_mqtt(int mqtt_port, uint8_t *mqtt_ip_addr, bool wifi_auto_reconnect = false) : mqtt_client(), mqtt_ip_addr(mqtt_ip_addr[0], mqtt_ip_addr[1], mqtt_ip_addr[2], mqtt_ip_addr[3])
{
  this->mqtt_port = mqtt_port;
  set_up_mqtt_callbacks();
  mqtt_client.setServer(mqtt_ip_addr, mqtt_port);
}

void qos_mqtt::connect_mqtt()
{
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Reconnecting WiFi");
    delay(4000);
  }
  Serial.print("Connecting to MQTT...");
  mqtt_client.connect();
  while (!mqtt_connected)
  {
    delay(3000);
  }
}

void qos_mqtt::publish_mqtt(char *topic, uint8_t *message, int length, int QoS)
{
  if (!mqtt_connected)
  {
    connect_mqtt();
  }

  mqtt_client.publish(topic, QoS, false, message, length);
}

void qos_mqtt::subscribe_mqtt(char *topic, int QoS)
{
  if (!mqtt_connected)
  {
    connect_mqtt();
  }

  mqtt_client.subscribe(topic, QoS);
}

void qos_mqtt::unsubscribe_mqtt(char *topic)
{
  mqtt_client.unsubscribe(topic);
}

bool qos_mqtt::is_connected()
{
  return mqtt_connected;
}

void qos_mqtt::set_up_mqtt_callbacks()
{
  mqtt_client.onConnect(on_mqtt_connect);
  mqtt_client.onDisconnect(on_mqtt_disconnect);
  mqtt_client.onSubscribe(on_mqtt_subscribe);
  mqtt_client.onUnsubscribe(on_mqtt_unsubscribe);
  mqtt_client.onMessage(on_mqtt_message);
  mqtt_client.onPublish(on_mqtt_publish);
}

void qos_mqtt::on_mqtt_message(const espMqttClientTypes::MessageProperties &properties, const char *topic, const uint8_t *payload, size_t len, size_t index, size_t total)
{
  Serial.println("Message recieved");
  for (int i = 0; i < 25; i++)
  {
    last_message[i] = '\0';
  }
  strncpy(last_message, (char *)payload, len);
}

void qos_mqtt::on_mqtt_publish(uint16_t packetId)
{
}

void qos_mqtt::on_mqtt_unsubscribe(uint16_t packetId)
{
}

void qos_mqtt::on_mqtt_subscribe(uint16_t packetId, const espMqttClientTypes::SubscribeReturncode *codes, size_t len)
{
}

void qos_mqtt::on_mqtt_connect(bool sessionPresent)
{
  Serial.println("");
  Serial.println("Connected to MQTT");
  mqtt_connected = true;
}

void qos_mqtt::on_mqtt_disconnect(espMqttClientTypes::DisconnectReason reason)
{
  Serial.println("");
  Serial.println("Disconnected from MQTT");
  mqtt_connected = false;
}

void qos_mqtt::get_last_message(char *array)
{
  strncpy(array, last_message, 25);
  return;
}