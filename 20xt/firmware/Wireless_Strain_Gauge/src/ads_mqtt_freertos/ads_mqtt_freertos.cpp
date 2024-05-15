#include "ads_mqtt_freertos.h"

#define RTOS_QUEUE_SIZE 1000
#define MQTT_PORT 1883
#define MESSAGE_DATA_LENGTH 40
#define MQTT_MESSAGE_LENGTH MESSAGE_DATA_LENGTH * 2 + 4
#define MQTT_SEND_MESSAGE_THRESHHOLD RTOS_QUEUE_SIZE / 100
#define CS_PIN 5
#define INTRUPT_PIN 4
#define MQTT_QoS 2

namespace crt
{
  typedef struct message
  {
    uint32_t milliseconds;
    uint16_t data[MESSAGE_DATA_LENGTH];
  } message;

  xQueueHandle data_queue = xQueueCreate(RTOS_QUEUE_SIZE, sizeof(message));
  bool send_flag = false;

  send_value::send_value(const char *task_name, unsigned int task_priority, unsigned int task_size_bytes, unsigned int task_core_number, uint8_t *broker_ip_address) : Task(task_name, task_priority, task_size_bytes, task_core_number)
  {
    this->broker_ip_address = broker_ip_address;
    start();
  }

  void send_value::StaticMain(void *pParam)
  {
    send_value *THIS = (send_value *)pParam;
    THIS->main();
  }

  void send_value::main()
  {
    uint8_t msg[MESSAGE_DATA_LENGTH * 2 + 4];
    message recieved;

    char publish_topic[25];
    char mqtt_message[25];
    char subscribe_topic[] = "esp32/send";
    create_publish_topic(publish_topic);

    qos_mqtt mqtt_client(MQTT_PORT, broker_ip_address, true);
    mqtt_client.connect_mqtt();
    mqtt_client.subscribe_mqtt(subscribe_topic, MQTT_QoS);

    for (;;)
    {
      mqtt_client.get_last_message(mqtt_message);
      if ((strcmp(mqtt_message, "send") == 0) && mqtt_client.is_connected()) {
        send_flag = true;
      } else {
        send_flag = false;
        if (!mqtt_client.is_connected()) {
          mqtt_client.connect_mqtt();
          mqtt_client.subscribe_mqtt(subscribe_topic, MQTT_QoS);        
        }
      }

      if (uxQueueMessagesWaiting(data_queue) >= MQTT_SEND_MESSAGE_THRESHHOLD)
      {
        Serial.println("Sending...");
        for (int i = 0; i < MQTT_SEND_MESSAGE_THRESHHOLD; i++)
        {
          xQueueReceive(data_queue, &recieved, 1000);
          copy_time_to_message(msg, recieved.milliseconds);
          copy_data_to_message(msg, recieved.data);
          mqtt_client.publish_mqtt(publish_topic, msg, MQTT_MESSAGE_LENGTH, MQTT_QoS);
          vTaskDelay(10);
        }
      }
      vTaskDelay(200);
    }
  }

  void send_value::create_publish_topic(char *publish_topic)
  {
    char topicHeader[] = "esp32/";
    const char *topicFooter = WiFi.macAddress().c_str();
    strcpy(publish_topic, topicHeader);
    strcat(publish_topic, topicFooter);
    Serial.println(publish_topic);
  }

  void send_value::copy_time_to_message(uint8_t *message, uint32_t time)
  {
    message[0] = (time & 0xff000000) >> 24;
    message[1] = (time & 0x00ff0000) >> 16;
    message[2] = (time & 0x0000ff00) >> 8;
    message[3] = time & 0x000000ff;
  }

  void send_value::copy_data_to_message(uint8_t *message, uint16_t *data)
  {
    for (int i = 0; i < MESSAGE_DATA_LENGTH; i++)
    {
      message[2 * i + 4] = (data[i] >> 8);
      message[2 * i + 5] = (data[i] & 0xff);
    }
  }

  void record_value::StaticMain(void *pParam)
  {
    record_value *THIS = (record_value *)pParam;
    THIS->main();
  }

  record_value::record_value(const char *task_name, unsigned int task_priority, unsigned int task_size_bytes, unsigned int task_core_number) : Task(task_name, task_priority, task_size_bytes, task_core_number)
  {
    start(); // starts main()
  }

  void record_value::main()
  {
    uint16_t value = 0;
    int num_values = 0;
    while (send_flag == false) {
      vTaskDelay(1);
    }
    ESP32Time rtc(0);

    ads_1120 ads1120 = ads_1120();
    message msg;
    message dump;

    ads1120.begin(CS_PIN, INTRUPT_PIN);
    Serial.println("ADS started");
    rtc.setTime(0, 0, 1, 1, 1, 2021);
    msg.milliseconds = get_rtc_millis(rtc);
    uint32_t last_time = get_rtc_millis(rtc);

    for (;;)
    {
      value = ads1120.read_adc();
      if (value != NULL)
      {
        msg.data[num_values] = value;
        num_values++;
        if (num_values % MESSAGE_DATA_LENGTH == 0)
        {
          num_values = 0;
          if (!uxQueueSpacesAvailable(data_queue))
          {
            xQueueReceive(data_queue, &dump, 1000);
          }
          xQueueSend(data_queue, &msg, 1000);
          msg.milliseconds = get_rtc_millis(rtc);
        }
      }

      if (get_rtc_millis(rtc) - last_time > 30000 && !send_flag) {
        Serial.println("RecordValue: Waiting for flag");
        while (send_flag == false) {
          vTaskDelay(1);
        }
        Serial.println("RecordValue: Flag set");
        last_time = get_rtc_millis(rtc);
      }
      vTaskDelay(1);
    }
  }

  uint32_t record_value::get_rtc_millis(ESP32Time &rtc)
  {
    return rtc.getMillis() + rtc.getSecond() * 1000 + rtc.getMinute() * 60000 + (rtc.getHour() - 1) * 3600000;
  }
};
