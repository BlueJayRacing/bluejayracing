/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include "WiFiMQTT.h"

char topic[] = "testTopic1";
int count = 0;
int totCount = 0;
uint8_t msg[150];
unsigned int length = 100;
char ssid[] = "piWifi";
char pswd[] = "bluejayracing";
char ip[] = "10.42.0.1";

WiFiMQTT mqttSubscriber(ssid, pswd, ip, 1883);

void setup() {
  Serial.begin(115200);
  mqttSubscriber.beginWifi();
  mqttSubscriber.beginMQTT();
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
  
  mqttSubscriber.sendMQTTMessage(topic, msg, length);
}