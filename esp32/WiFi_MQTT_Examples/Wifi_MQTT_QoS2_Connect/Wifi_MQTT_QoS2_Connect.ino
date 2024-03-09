#include <WiFi.h>

#include "QoSWiFiMQTT.h"

#define wifi_ssid "piWifi"
#define wifi_pswd "bluejayracing"
#define mqtt_port 1883
#define wifi_auto_connect true
int count = 0;
int totCount = 0;
char msg[150];
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
  for (int i = 0; i < 100; i++){
    msg[i] = count + 'A';
    count ++;
    count = count % 26;
    totCount++;
    if (totCount % 10000 == 0){
      Serial.println(millis());
    }
  }

  mqtt_client->publishMQTT(topic, msg, 2);
}

//65000
//56925
//24719