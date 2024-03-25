#include "WiFiFreeRtosClient.h"

#define RTOS_QUEUE_SIZE 50000
#define MQTT_PORT 1883
#define MQTT_MESSAGE_LENGTH 50
#define MQTT_SEND_MESSAGE_THRESHHOLD RTOS_QUEUE_SIZE / 10
#define CS_PIN 5
#define INTRUPT_PIN 2

namespace crt
{
  //global data queue that is to be shared by the sendValue and recordValue class
  xQueueHandle dataQueue = xQueueCreate(RTOS_QUEUE_SIZE, sizeof(uint16_t));

  sendValue::sendValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber, char* ssid, char* pswd, uint8_t* ip) :
	Task(taskName, taskPriority, taskSizeBytes, taskCoreNumber)
	{
    //creates a new BLECLIENT and starts the freeRTOS Task
    this->mqtt_client = new QoSWiFiMQTT(ssid, pswd, MQTT_PORT, ip, true);
    this->mqtt_client->connectToWiFi();
    this->mqtt_client->connectToMQTT();
    uint8_t arr[4] = {1, 2, 3, 4};
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
    uint8_t msg[MQTT_MESSAGE_LENGTH * 2];
    uint16_t recieved;
    uint8_t leftByte;
    uint8_t rightByte;

    for(;;) {
      if (uxQueueMessagesWaiting(dataQueue) >= MQTT_SEND_MESSAGE_THRESHHOLD){
        Serial.println("Sending...");
        for (int i = 0; i < MQTT_SEND_MESSAGE_THRESHHOLD / MQTT_MESSAGE_LENGTH + 1; i++) {
          for (int j = 0; j < MQTT_MESSAGE_LENGTH; j++){
            xQueueReceive(dataQueue, &recieved, 1000);
            leftByte = (recieved & 0xff);
            rightByte = (recieved >> 8);
            msg[2*j] = rightByte;
            msg[2*j + 1] = leftByte;
          }
          
          mqtt_client->publishMQTT(topic, msg, MQTT_MESSAGE_LENGTH * 2, 2);
          delay(3);
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
		start();
	}

  //The FREERTOS Task that the recordValue is constantly running
	void recordValue::main(){
    uint16_t value = 0;
    int numValues = 0;
    Serial.println("RecordValue main called");
		vTaskDelay(10000); // wait for other threads to have started up as well.
    
		xADS1120 ads1120 = xADS1120();

    ads1120.begin(CS_PIN, INTRUPT_PIN);
    Serial.println("ADS started");

		for (;;){
      value = ads1120.readADC();
      if (value != NULL){
        numValues ++;
        if (numValues % 2000 == 0){
          Serial.println(millis());
        }
        xQueueSend(dataQueue, &value, 1000);
      }
    }
	}
};// end namespace crt
