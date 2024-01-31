#include <BLEDevice.h>
#include <string.h>
#include <iostream>
#include "Arduino.h"


#ifndef ble_client.h
#define ble_client.h
//Callback for BleClient object when it connects/disconnects from a server
class MyClientCallback : public BLEClientCallbacks {
  public:
    MyClientCallback(bool& connected);
    void onConnect(BLEClient* pclient);
    void onDisconnect(BLEClient* pclient);
  private:
    bool* connected;
};
//Callback for bleScan when BLEScan finds a new BLE devie
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks{
  public:
    MyAdvertisedDeviceCallbacks(BLEAdvertisedDevice*& myDevice, bool& doConnect, bool& doScan, char* address);
    void onResult(BLEAdvertisedDevice advertisedDevice);
  private:
    BLEAdvertisedDevice** myDevice;
    bool* doConnect;
    bool* doScan;
    char* desiredAddress;
};

class BLECLIENT
{
  public:
    BLECLIENT(char* address, char* serviceUUID, char* charUUID);
    char* obtainAddress();
    BLEUUID obtainServiceUUID();
    BLEUUID obtaincharUUID();
    void startBLE();
    bool isConnected();
    bool connectToServer();
    BLERemoteCharacteristic* getDataCharacteristic();
  private:
    char* address;
    BLEUUID serviceUUID;
    BLEUUID charUUID;
    static bool doConnect;
    static bool connected;
    static bool doScan;
    static bool changed;
    static BLEAdvertisedDevice* myDevice;
    BLERemoteCharacteristic* data_charac;
    bool connectCharacteristic(BLERemoteService*& pRemoteService,
                        BLERemoteCharacteristic*& l_BLERemoteChar);
};

#endif