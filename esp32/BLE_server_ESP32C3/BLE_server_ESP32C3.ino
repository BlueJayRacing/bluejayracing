/*
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
  Ported to Arduino ESP32 by Evandro Copercini
  updated by chegewara and MoThunderz
  actual file
*/
#include "ble_server.h"

void setup(){
  Serial.begin(115200);
  //keep a delay function here to prevent the serial port from being spammed
  //if the ESP32 is constantly rebooting
  delay(1000);

  //Define the service UUID and the characteristic UUID of the server
  char serviceUUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  char charUUID[] = "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e";

  //Create the server object by passing in the service and characteristic UUID
  BLESERVER* bleServer = new BLESERVER(serviceUUID, charUUID);

  //Start the Advertising for the server; advertising will stop after one device connects,
  //and it will automatically resume when a device disconnects
  bleServer->startAdvertising();
}

void loop(){
  delay(1000);
}