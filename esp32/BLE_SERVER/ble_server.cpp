#include "ble_server.h"
#include <iostream>
#include "Arduino.h"

BLESERVER::BLESERVER(char* serviceUUID, char* charUUID){
  //Initialize class variables
  this->numConnected = 0;
  this->serviceUUID = serviceUUID;
  this->charUUID = charUUID;
  
  BLEDevice::init("ESP32");

  //Create a server object and set "MyServerCallbacks" as the callback class
  this->bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks(numConnected, bleAdvertising));

  //Create a service object linked to the server 
  //and create a characteristic linked to that service
  this->bleService = bleServer->createService(serviceUUID);
  this->data_Charac = bleService->createCharacteristic(
                      charUUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |                      
                      BLECharacteristic::PROPERTY_NOTIFY
                    );  

  //Create a characteristic descriptor and add it to the characteristic
  //This descriptor enables notifications (if you want notifications)
  data_Descr = new BLE2902();
  data_Descr->setNotifications(true);
  data_Charac->addDescriptor(data_Descr);

  //Set "MyCharacteristicCallbacks" as the callback class for data_Charac
  data_Charac->setCallbacks(new MyCharacteristicCallbacks());

  this->bleAdvertising = BLEDevice::getAdvertising();
}

//Call this function to initially start advertising the server to other devices
//ONLY CALL THIS ONCE; the callbacks should handle starting and stopping advertising on connect/disconnect
void BLESERVER::startAdvertising(){
  bleService->start();
  bleAdvertising->addServiceUUID(serviceUUID);
  bleAdvertising->setScanResponse(false);
  bleAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

int BLESERVER::getNumConnected(){
  return numConnected;
}

// Callback function that is called whenever a client is connected or disconnected
MyServerCallbacks::MyServerCallbacks(int& numConnected, BLEAdvertising* bleAdvertising){
  this->bleAdvertising = bleAdvertising;
  this->numConnected = &numConnected;  
}
  
void MyServerCallbacks::onConnect(BLEServer* pServer) {
  *numConnected = true;
  BLEDevice::stopAdvertising();
};

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  *numConnected = false;
  BLEDevice::startAdvertising();
}

//Callback function for when a client writes onto the server
void MyCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxValue = pCharacteristic->getValue();
  Serial.println("Message start----------");
  for (int i = 0; i < 20; i++){
    Serial.println((uint8_t)rxValue.c_str()[i]);
  }
}
