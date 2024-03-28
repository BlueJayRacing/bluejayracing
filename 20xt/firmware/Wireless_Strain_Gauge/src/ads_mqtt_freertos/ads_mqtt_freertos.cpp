#include "ads_mqtt_freertos.h"

#define RTOS_QUEUE_SIZE 50000
#define MQTT_PORT 1883
#define MQTT_MESSAGE_LENGTH 35
#define MQTT_SEND_MESSAGE_THRESHHOLD RTOS_QUEUE_SIZE / 200
#define CS_PIN 5
#define INTRUPT_PIN 4

namespace crt
{
  //global data queue that is to be shared by the sendValue and recordValue class
  xQueueHandle data_queue = xQueueCreate(RTOS_QUEUE_SIZE, sizeof(uint16_t));

  send_value::send_value(const char *task_name, unsigned int task_priority, unsigned int task_size_bytes, unsigned int task_core_number, uint8_t* broker_ip_address) :
	Task(task_name, task_priority, task_size_bytes, task_core_number) {
    this->broker_ip_address = broker_ip_address;
		start(); //starts main()
	}
	
   void send_value::StaticMain(void *pParam) {
		send_value* THIS = (send_value*) pParam;
		THIS->main();
	}

  void send_value::main() {
    char topic[] = "testTopic/4";
    uint8_t msg[MQTT_MESSAGE_LENGTH * 2 + 1];
    uint16_t recieved;
    uint8_t left_byte;
    uint8_t right_byte;
    uint8_t count = 0;

    qos_mqtt mqtt_client = qos_mqtt(MQTT_PORT, broker_ip_address, true);
    mqtt_client.connect_mqtt();

    for(;;) {
      if (uxQueueMessagesWaiting(data_queue) >= MQTT_SEND_MESSAGE_THRESHHOLD) {
        Serial.println("Sending...");
        for (int i = 0; i < MQTT_SEND_MESSAGE_THRESHHOLD / MQTT_MESSAGE_LENGTH + 1; i++) {
          count++;
          msg[0] = count;
          for (int j = 0; j < MQTT_MESSAGE_LENGTH; j++){
            xQueueReceive(data_queue, &recieved, 1000);
            left_byte = (recieved & 0xff);
            right_byte = (recieved >> 8);
            msg[2*j + 1] = right_byte;
            msg[2*j + 2] = left_byte;
          }
          
          mqtt_client.publish_mqtt(topic, msg, MQTT_MESSAGE_LENGTH * 2 + 1, 2);
          vTaskDelay(10);
        }
      }
      vTaskDelay(200);
    }
  }

  void record_value::StaticMain(void *pParam) {
		record_value* THIS = (record_value*) pParam;
		THIS->main();
	}

	record_value::record_value(const char *task_name, unsigned int task_priority, unsigned int task_size_bytes, unsigned int task_core_number) :
	Task(task_name, task_priority, task_size_bytes, task_core_number) {
		start(); //starts main()
	}

	void record_value::main() {
    uint16_t value = 0;
    int num_values = 0;
		vTaskDelay(10000); // Wait for other threads to start up
    
		ads_1120 ads1120 = ads_1120();

    ads1120.begin(CS_PIN, INTRUPT_PIN);
    Serial.println("ADS started");

		for (;;){
      value = ads1120.read_adc();
      if (value != NULL){
        num_values ++;
        if (num_values % 2000 == 0){
          Serial.println(millis());
        }
        xQueueSend(data_queue, &value, 1000);
      }
      vTaskDelay(1);
    }
	}
};// end namespace crt
