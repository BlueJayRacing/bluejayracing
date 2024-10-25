#include "ads_mqtt_freertos.h"

#define RTOS_QUEUE_SIZE              200
#define MQTT_PORT                    1883
#define MQTT_MESSAGE_LENGTH          40
#define MQTT_SEND_MESSAGE_THRESHHOLD RTOS_QUEUE_SIZE / 20
#define CS_PIN                       5
#define INTRUPT_PIN                  4

namespace crt
{
typedef struct message {
    uint32_t milliseconds;
    uint16_t data[MQTT_MESSAGE_LENGTH];
} message;

// global data queue that is to be shared by the sendValue and recordValue class
xQueueHandle data_queue = xQueueCreate(RTOS_QUEUE_SIZE, sizeof(message));

send_value::send_value(const char* task_name, unsigned int task_priority, unsigned int task_size_bytes,
                       unsigned int task_core_number, uint8_t* broker_ip_address)
    : Task(task_name, task_priority, task_size_bytes, task_core_number)
{
    this->broker_ip_address = broker_ip_address;
    start(); // starts main()
}

void send_value::StaticMain(void* pParam)
{
    send_value* THIS = (send_value*)pParam;
    THIS->main();
}

void send_value::main()
{
    uint8_t msg[MQTT_MESSAGE_LENGTH * 2 + 4];
    message recieved;
    uint8_t left_byte;
    uint8_t right_byte;

    char topic[25];
    char topicHeader[]      = "esp32/";
    const char* topicFooter = WiFi.macAddress().c_str();
    strcpy(topic, topicHeader);
    strcat(topic, topicFooter);
    Serial.println(topic);

    qos_mqtt mqtt_client = qos_mqtt(MQTT_PORT, broker_ip_address, true);
    mqtt_client.connect_mqtt();

    for (;;) {
        if (uxQueueMessagesWaiting(data_queue) >= MQTT_SEND_MESSAGE_THRESHHOLD) {
            Serial.println("Sending...");
            for (int i = 0; i < MQTT_SEND_MESSAGE_THRESHHOLD; i++) {
                xQueueReceive(data_queue, &recieved, 1000);
                msg[0] = (recieved.milliseconds & 0xff000000) >> 24;
                msg[1] = (recieved.milliseconds & 0x00ff0000) >> 16;
                msg[2] = (recieved.milliseconds & 0x0000ff00) >> 8;
                msg[3] = recieved.milliseconds & 0x000000ff;
                for (int j = 0; j < MQTT_MESSAGE_LENGTH; j++) {
                    left_byte      = (recieved.data[j] & 0xff);
                    right_byte     = (recieved.data[j] >> 8);
                    msg[2 * j + 4] = right_byte;
                    msg[2 * j + 5] = left_byte;
                }

                mqtt_client.publish_mqtt(topic, msg, MQTT_MESSAGE_LENGTH * 2 + 4, 2);
                vTaskDelay(10);
            }
        }
        vTaskDelay(200);
    }
}

void record_value::StaticMain(void* pParam)
{
    record_value* THIS = (record_value*)pParam;
    THIS->main();
}

record_value::record_value(const char* task_name, unsigned int task_priority, unsigned int task_size_bytes,
                           unsigned int task_core_number)
    : Task(task_name, task_priority, task_size_bytes, task_core_number)
{
    start(); // starts main()
}

void record_value::main()
{
    uint16_t value = 0;
    int num_values = 0;
    vTaskDelay(10000); // Wait for other threads to start up
    ESP32Time rtc(0);

    ads_1120 ads1120 = ads_1120();
    message msg;

    ads1120.begin(CS_PIN, INTRUPT_PIN);
    Serial.println("ADS started");
    rtc.setTime(0, 0, 1, 1, 1, 2021);
    msg.milliseconds =
        rtc.getMillis() + rtc.getSecond() * 1000 + rtc.getMinute() * 60000 + (rtc.getHour() - 1) * 3600000;

    for (;;) {
        value = ads1120.read_adc();
        if (value != NULL) {
            msg.data[num_values] = value;
            num_values++;
            if (num_values % MQTT_MESSAGE_LENGTH == 0) {
                num_values = 0;
                xQueueSend(data_queue, &msg, 1000);
                msg.milliseconds =
                    rtc.getMillis() + rtc.getSecond() * 1000 + rtc.getMinute() * 60000 + (rtc.getHour() - 1) * 3600000;
            }
        }
        vTaskDelay(1);
    }
}
}; // end namespace crt
