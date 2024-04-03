#include "threads/ads_mqtt_freertos/ads_mqtt_freertos.h"
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  char ssid[] = "piWifi";
  char pswd[] = "bluejayracing";
  uint8_t broker_ip_address[] = {10, 42, 0, 1};
  WiFi.begin(ssid, pswd); 
  WiFi.setAutoReconnect(true);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Still connecting");
    delay(4000);
  }

  //Create a recordValue object which records a value to be sent by the sendValue object
  crt::record_value recordValueTask("recordValue", 2 /*priority*/, 40000 /*stackBytes*/, ARDUINO_RUNNING_CORE);
  //Create a sendValue object which creates a BLECLIENT object and a thread to send values to that client 
  crt::send_value sendValueTask("sendValue", 2 /*priority*/, 40000 /*stackBytes*/, ARDUINO_RUNNING_CORE, broker_ip_address);
}

void loop() {
  delay(1000);
}
