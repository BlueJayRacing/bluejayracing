#include "WiFiFreeRtosClient.h"

namespace crt
{
  //global data queue that is to be shared by the sendValue and recordValue class
  xQueueHandle data_queue = xQueueCreate(20000, sizeof(uint16_t));

  sendValue::sendValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber, char* ssid, char* pswd, char* ip) :
	Task(taskName, taskPriority, taskSizeBytes, taskCoreNumber)
	{
    //creates a new BLECLIENT and starts the freeRTOS Task
    this->mqttSubscriber = new WiFiMQTT(ssid, pswd, ip, 1883);
    this->mqttSubscriber->beginWifi();
    this->mqttSubscriber->beginMQTT();
		start();
	}
	
   void sendValue::StaticMain(void *pParam)
	{
		sendValue* THIS = (sendValue*) pParam;
		THIS->main();
	}

  //The FreeRTOS Task for sendValue that is constantly running through the for loop
  void sendValue::main(){
    char topic[] = "testTopic";
    uint8_t msg[150];
    unsigned int length = 100;
    uint16_t recieved = 0;
    uint8_t leftByte;
    uint8_t rightByte;

    for(;;) {
      if (uxQueueMessagesWaiting(data_queue) >= 5000){
        for (int i = 0; i < 100; i++) {
          for (int j = 0; j < 50; j++){
            xQueueReceive(data_queue, &recieved, 1000);
            leftByte = ((uint8_t) recieved & 0xff);
            rightByte = ((uint8_t) recieved >> 8);
            msg[2*j] = rightByte;
            msg[2*j + 1] = leftByte;
          }
          
          mqttSubscriber->sendMQTTMessage(topic, msg, length);
        }
      }
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
    
		xADS1115 ads1115 = xADS1115(2, 16, 7);

    ads1115.beginADS();
    Serial.println("ADS started");

		for (;;){
      value = abs(ads1115.handleConversion());
      if (value != NULL){
        count ++;
        if (count % 1720 == 0){
          Serial.println(millis());
        }
        xQueueSend(data_queue, &value, 1000);
      }
    }
	}
};// end namespace crt
