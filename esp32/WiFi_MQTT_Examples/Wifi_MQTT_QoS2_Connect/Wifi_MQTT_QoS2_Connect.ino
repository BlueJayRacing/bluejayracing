#include <WiFi.h>

#include "QoSWiFiMQTT.h"

#define wifi_ssid "piWifi"
#define wifi_pswd "bluejayracing"
#define mqtt_port 1883
#define wifi_auto_connect true
int count = 0;
int totCount = 0;
int msgCount = 0;
uint8_t msg[100];
char topic[] = "testTopic/4";

QoSWiFiMQTT* mqtt_client;

void setup() {
  Serial.begin(115200);
  Serial.println();

  uint8_t mqtt_addr[] = {10, 42, 0, 1};
  
  mqtt_client = new QoSWiFiMQTT(wifi_ssid, wifi_pswd, mqtt_port, mqtt_addr, wifi_auto_connect);
  mqtt_client->connectToWiFi();
  mqtt_client->connectToMQTT();
}

void loop() {
  msg[0] = msgCount;
  for (int i = 1; i < 100; i++){
    msg[i] = count;
    count ++;
    totCount++;
  }
  msgCount++;

  mqtt_client->publishMQTT(topic, msg, 100, 2);
  delay(3);
}

//65000
//56925
//24719