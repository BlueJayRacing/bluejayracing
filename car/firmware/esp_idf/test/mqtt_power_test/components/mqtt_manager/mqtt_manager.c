#include <stdio.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#include "mqtt_manager.h"

static const char* TAG = "mqtt_manager";

#define MQTT_PUBLISH_TOPIC           "esp32/data"
#define MQTT_SUBSCRIBE_TOPIC         "esp32/can_send"
#define MQTT_PORT                    1883
#define MQTT_SEND_MESSAGE_THRESHHOLD RTOS_QUEUE_SIZE / 20

static esp_mqtt_client_handle_t mqtt_handle;

static bool is_mqtt_connected = false;

static bool can_send_received;

static QueueHandle_t time_queue = NULL;

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data)
{
    esp_mqtt_event_handle_t event   = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        esp_mqtt_client_subscribe(client, MQTT_SUBSCRIBE_TOPIC, 2);
        is_mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        is_mqtt_connected = false;
        break;
    case MQTT_EVENT_SUBSCRIBED:
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        break;
    case MQTT_EVENT_PUBLISHED:
        break;
    case MQTT_EVENT_DATA:
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        if (memcmp("SEND", event->data, sizeof("SEND") - 1) == 0) {
            can_send_received = true;
            mqtt_time_t pi_time;

            pi_time.esp_time_micro = esp_timer_get_time();
            memcpy(&pi_time.pi_time_micro, event->data + sizeof("SEND") - 1, sizeof(uint64_t));

            if (uxQueueSpacesAvailable(time_queue) == 0) {
                mqtt_time_t dis_time;
                xQueueReceive(time_queue, &dis_time, (TickType_t)1);
            }

            xQueueSend(time_queue, &pi_time, (TickType_t)1);
        } else if (memcmp("NOSEND", event->data, sizeof("NOSEND")) == 0) {
            can_send_received = false;
        }

        break;
    case MQTT_EVENT_ERROR:
        break;
    default:
        break;
    }
}

esp_err_t mqtt_start(void)
{
    esp_mqtt_client_destroy(mqtt_handle);

    time_queue = xQueueCreate(2, sizeof(mqtt_time_t));
    if (time_queue == NULL) {
        ESP_LOGE(TAG, "Failed to initialize time queue\n");
    }

    esp_mqtt_client_config_t mqtt_config;
    memset(&mqtt_config, 0, sizeof(esp_mqtt_client_config_t));

    mqtt_config.broker.address.uri = "mqtt://10.42.0.1";

    mqtt_handle = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_handle == NULL) {
        return ESP_ERR_NOT_FINISHED;
    }

    esp_err_t err = esp_mqtt_client_register_event(mqtt_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_mqtt_client_start(mqtt_handle);
    if (err != ESP_OK) {
        return err;
    }

    while (!is_mqtt_connected) {
        vTaskDelay(100);
    }

    return ESP_OK;
}

esp_err_t mqtt_update_time(mqtt_time_t* ptr_time)
{
    if (uxQueueMessagesWaiting(time_queue) > 0) {
        xQueueReceive(time_queue, ptr_time, (TickType_t)1);
        return ESP_OK;
    }

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t mqtt_send_message(const char* msg, const uint16_t msg_length)
{
    return esp_mqtt_client_publish(mqtt_handle, MQTT_PUBLISH_TOPIC, msg, msg_length, 0, 0);
}

bool mqtt_can_send_received(void) { return can_send_received; }
