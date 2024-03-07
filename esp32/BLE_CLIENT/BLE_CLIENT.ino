#include "initFreeRtosClient.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  //Define the service UUID, characteristic UUID, and address UUID of the server that you wish to connect to
  char serviceUUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  char charUUID[] = "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e";
  char address[] = "84:fc:e6:00:92:de";

  //Create a sendValue object which creates a BLECLIENT object and a thread to send values to that client 
  crt::sendValue sendValueTask("sendValue", 2 /*priority*/, 40000 /*stackBytes*/, ARDUINO_RUNNING_CORE, address, serviceUUID, charUUID);
  //Create a recordValue object which records a value to be sent by the sendValue object
  crt::recordValue recordValueTask("recordValue", 2 /*priority*/, 40000 /*stackBytes*/, ARDUINO_RUNNING_CORE);
}

void loop() {
}
