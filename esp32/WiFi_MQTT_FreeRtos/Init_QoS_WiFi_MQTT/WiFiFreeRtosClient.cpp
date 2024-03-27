#include "esp32-hal.h"
#include "WiFiFreeRtosClient.h"

#define RTOS_QUEUE_SIZE 50000
#define MQTT_PORT 1883
#define MQTT_MESSAGE_LENGTH 35
#define MQTT_SEND_MESSAGE_THRESHHOLD RTOS_QUEUE_SIZE / 200
#define CS_PIN 5
#define INTRUPT_PIN 4

namespace crt
{
  //global data queue that is to be shared by the sendValue and recordValue class
  xQueueHandle dataQueue = xQueueCreate(RTOS_QUEUE_SIZE, sizeof(uint16_t));

  sendValue::sendValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber, uint8_t* ip) :
	Task(taskName, taskPriority, taskSizeBytes, taskCoreNumber) {
    this->ip_address = ip;
		start(); //starts main()
	}
	
   void sendValue::StaticMain(void *pParam) {
		sendValue* THIS = (sendValue*) pParam;
		THIS->main();
	}

  void sendValue::main() {
    char topic[] = "testTopic/4";
    uint8_t msg[MQTT_MESSAGE_LENGTH * 2 + 1];
    uint16_t recieved;
    uint8_t leftByte;
    uint8_t rightByte;
    uint8_t count = 0;

    QoSWiFiMQTT mqtt_client = QoSWiFiMQTT(MQTT_PORT, ip_address, true);
    mqtt_client.connectToMQTT();

    for(;;) {
      if (uxQueueMessagesWaiting(dataQueue) >= MQTT_SEND_MESSAGE_THRESHHOLD) {
        Serial.println("Sending...");
        for (int i = 0; i < MQTT_SEND_MESSAGE_THRESHHOLD / MQTT_MESSAGE_LENGTH + 1; i++) {
          count++;
          msg[0] = count;
          for (int j = 0; j < MQTT_MESSAGE_LENGTH; j++){
            xQueueReceive(dataQueue, &recieved, 1000);
            leftByte = (recieved & 0xff);
            rightByte = (recieved >> 8);
            msg[2*j + 1] = rightByte;
            msg[2*j + 2] = leftByte;
          }
          
          mqtt_client.publishMQTT(topic, msg, MQTT_MESSAGE_LENGTH * 2 + 1, 2);
          vTaskDelay(10);
        }
      }
      vTaskDelay(200);
    }
  }

  void recordValue::StaticMain(void *pParam) {
		recordValue* THIS = (recordValue*) pParam;
		THIS->main();
	}

	recordValue::recordValue(const char *taskName, unsigned int taskPriority, unsigned int taskSizeBytes, unsigned int taskCoreNumber) :
	Task(taskName, taskPriority, taskSizeBytes, taskCoreNumber) {
		start(); //starts main()
	}

	void recordValue::main() {
    uint16_t value = 0;
    int numValues = 0;
		vTaskDelay(10000); // Wait for other threads to start up
    
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
      vTaskDelay(1);
    }
	}
};// end namespace crt
