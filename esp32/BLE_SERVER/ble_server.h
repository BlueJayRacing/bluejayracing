#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//Class that is automatically called when a client connects/disconnects from the server
class MyServerCallbacks: public BLEServerCallbacks {
    public:
        MyServerCallbacks(int& connected, BLEAdvertising* bleAdvertising);
        void onConnect(BLEServer* pServer);
        void onDisconnect(BLEServer* pServer);
    private:
      int* numConnected;
      BLEAdvertising* bleAdvertising;
};

//Class that is automatically called when a client does some action to a characteristic
//Ex: read, write, etc. 
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    public:
        void onWrite(BLECharacteristic *pCharacteristic);
};

//Class that represents the server object
class BLESERVER{
    public:
        BLESERVER(char* serviceUUID, char* charUUID);
        void startAdvertising();
        int getNumConnected();
    private:
        char* serviceUUID;
        char* charUUID; 
        BLEServer* bleServer;
        BLEService* bleService;
        BLECharacteristic* data_Charac;
        BLEAdvertising* bleAdvertising;
        BLE2902* data_Descr;
        int numConnected;
};