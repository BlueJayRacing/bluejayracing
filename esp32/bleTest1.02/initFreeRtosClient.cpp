#include "initFreeRtosClient.h"

namespace crt
{
  //global data queue that is to be shared by the sendValue and recordValue class
  xQueueHandle data_queue = xQueueCreate(4000, sizeof(uint16_t));

  sendValue::sendValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber, char* address, char* serviceUUID, char* charUUID) :
	Task(taskName, taskPriority, taskSizeBytes, taskCoreNumber)
	{
    //creates a new BLECLIENT and starts the freeRTOS Task
    this->bleClient = new BLECLIENT(address, serviceUUID, charUUID);
    bleClient->startBLE();
		start();
	}
	
   void sendValue::StaticMain(void *pParam)
	{
		sendValue* THIS = (sendValue*) pParam;
		THIS->main();
	}

  //The FreeRTOS Task for sendValue that is constantly running through the for loop
  void sendValue::main(){
    uint16_t recieved = 0;
    uint8_t dataArray[20];
    uint8_t leftByte;
    uint8_t rightByte;
    Serial.println("sendValue main called");
    vTaskDelay(10000); // wait for other threads to have started up as well.
    Serial.println("Prepare send values");
    for (;;){
      if (uxQueueMessagesWaiting(data_queue) >= 1600){
        if (!(bleClient->isConnected())){
          bleClient->connectToServer();
        }
        for(int i = 0; i < 60; i ++){
          for(int j = 0; j < 10; j ++){
            xQueueReceive(data_queue, &recieved, 1000);

            leftByte = ((uint8_t) recieved & 0xff);
            rightByte = ((uint8_t) recieved >> 8);
            dataArray[2*j] = rightByte;
            dataArray[2*j + 1] = leftByte;
            /*
            Serial.println(recieved);
            Serial.println(leftByte);
            Serial.println(rightByte);
            */
          }
          bleClient->getDataCharacteristic()->writeValue(dataArray, 20);
          vTaskDelay(10);
        }
      }
      vTaskDelay(200);
    }
  }//end main

  void recordValue::StaticMain(void *pParam)
	{
		recordValue* THIS = (recordValue*) pParam;
		THIS->main();
	}

	recordValue::recordValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber) :
	Task(taskName, taskPriority, taskSizeBytes, taskCoreNumber)
	{
    this->count = 0;
		start();
	}

  //The FREERTOS Task that the recordValue is constantly running
	void recordValue::main(){
    uint16_t value = 0;
    int count = 0;
    Serial.println("RecordValue main called");
		vTaskDelay(10000); // wait for other threads to have started up as well.
    
		bleADS1115 ads1115 = bleADS1115(2, 16, 7);

    ads1115.beginADS();
    Serial.println("ADS started");

		for (;;){
      value = abs(ads1115.handleConversion());
      if (value != NULL){
        value += 256;
        count ++;
        if (count % 10000 == 0){
          Serial.println(millis());
        }
        xQueueSend(data_queue, &value, 1000);
      }
    }
	}
};// end namespace crt
