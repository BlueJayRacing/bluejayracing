#include "WiFiFreeRtosClient.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  char ssid[] = "piWifi";
  char pswd[] = "bluejayracing";
  uint8_t ip[] = {10, 42, 0, 1};
  //Create a recordValue object which records a value to be sent by the sendValue object
  crt::recordValue recordValueTask("recordValue", 3 /*priority*/, 40000 /*stackBytes*/, ARDUINO_RUNNING_CORE);
  //Create a sendValue object which creates a BLECLIENT object and a thread to send values to that client 
  crt::sendValue sendValueTask("sendValue", 2 /*priority*/, 40000 /*stackBytes*/, ARDUINO_RUNNING_CORE, ssid, pswd, ip);
}

void loop() {
  delay(1);
}
