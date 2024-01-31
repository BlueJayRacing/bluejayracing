#include "ble_client.h"

MyClientCallback::MyClientCallback(bool& connected){
  this->connected = &connected;
}

void MyClientCallback::onConnect(BLEClient* pclient){}
void MyClientCallback::onDisconnect(BLEClient* pclient){
  *connected = false;
  Serial.println("onDisconnect");
}

MyAdvertisedDeviceCallbacks::MyAdvertisedDeviceCallbacks(BLEAdvertisedDevice*& myDevice, bool& doConnect, bool& doScan, char* address){
  this->myDevice = &myDevice;
  this->doConnect = &doConnect;
  this->doScan = &doScan;
  this->desiredAddress = address;
}

// Scan for BLE servers and find the first one that advertises the service we are looking for.
void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice)
{
  Serial.print("BLE Advertised Device found: ");
  Serial.println(advertisedDevice.toString().c_str());
  Serial.println(advertisedDevice.getAddress().toString().c_str());
  
  // We have found a device, let us now see if it contains the service we are looking for by comparing the addresses
  if (strcmp(advertisedDevice.getAddress().toString().c_str(), desiredAddress) == 0)
  {
    Serial.println("myDevice Assigned");
    BLEDevice::getScan()->stop();
    BLEAdvertisedDevice* thing = new BLEAdvertisedDevice(advertisedDevice);
    *myDevice = thing;
    *doConnect = true;
    *doScan = true;
  } // Found our server
}

bool BLECLIENT::doConnect = false;
bool BLECLIENT::connected = false;
bool BLECLIENT::doScan = false;
bool BLECLIENT::changed = false;
BLEAdvertisedDevice* BLECLIENT::myDevice = NULL;

BLECLIENT::BLECLIENT(char* newaddress, char* newserviceUUID, char* newcharUUID){
  address = newaddress;
  serviceUUID = BLEUUID(newserviceUUID);
  charUUID = BLEUUID(newcharUUID);
}

char* BLECLIENT::obtainAddress(){
  return address;
}

BLEUUID BLECLIENT::obtainServiceUUID(){
  return serviceUUID;
}

BLEUUID BLECLIENT::obtaincharUUID(){
  return charUUID;
}

bool BLECLIENT::isConnected(){
  return connected;
}

BLERemoteCharacteristic* BLECLIENT::getDataCharacteristic(){
  return data_charac;
}

// Function to chech Characteristic
bool BLECLIENT::connectCharacteristic(BLERemoteService*& pRemoteService, BLERemoteCharacteristic*& l_BLERemoteChar) {
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  if (l_BLERemoteChar == nullptr)
  {
    Serial.print("Failed to find one of the characteristics");
    Serial.print(l_BLERemoteChar->getUUID().toString().c_str());
    return false;
  }
  Serial.println(" - Found characteristic: " + String(l_BLERemoteChar->getUUID().toString().c_str()));
  /*
  if(l_BLERemoteChar->canNotify())
    l_BLERemoteChar->registerForNotify(notifyCallback);
  */
  return true;
}

// Function that is run whenever the server is connected
bool BLECLIENT::connectToServer()
{
  Serial.println(doConnect);
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  
  BLEClient* pClient = BLEDevice::createClient();

  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback(connected));

  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  delay(200);

  // Obtain a reference to the service we are after in the remote BLE server.
  Serial.println(serviceUUID.toString().c_str());
  BLERemoteService* pRemoteService = pClient->getService(this->serviceUUID);

  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  connected = true;
  data_charac = pRemoteService->getCharacteristic(charUUID);

  if(connectCharacteristic(pRemoteService, data_charac) == false)
    connected = false;

  if(connected == false) {
    pClient-> disconnect();
    Serial.println("At least one characteristic UUID not found");
    return false;
  }
  return true;
}

//function to start BLE for the BLECLIENT
void BLECLIENT::startBLE() {
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 40 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(myDevice, doConnect, doScan, address));
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(40, false);

  //Connect to the server refernce stored in myDevice after pBLEScan finishes
  connectToServer();
}